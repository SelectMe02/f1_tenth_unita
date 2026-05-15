#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path
from copy import deepcopy

import numpy as np
import rclpy
from std_msgs.msg import String, Float32
from f110_msgs.msg import Wpnt, WpntArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf_transformations import quaternion_from_euler

from global_planner.readwrite_global_waypoints import write_global_waypoints


def read_points(csv_path: Path):
    pts = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row["x"]), float(row["y"])])

    if len(pts) < 3:
        raise RuntimeError("manual_waypoints.csv에는 최소 3개 이상의 점이 필요합니다.")

    # 너무 가까운 중복 점 제거
    filtered = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - filtered[-1][0], p[1] - filtered[-1][1]) > 0.03:
            filtered.append(p)

    pts = filtered

    # closed loop 처리
    if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) > 0.2:
        pts.append(pts[0])

    return np.array(pts, dtype=float)


def resample_closed_path(points, ds):
    # 마지막 점은 첫 점과 같은 closed point라고 가정
    seg = points[1:] - points[:-1]
    seg_len = np.linalg.norm(seg, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg_len)])
    total_len = s[-1]

    if total_len < 1.0:
        raise RuntimeError("경로 길이가 너무 짧습니다.")

    sample_s = np.arange(0.0, total_len, ds)
    out = []

    for ss in sample_s:
        idx = np.searchsorted(s, ss, side="right") - 1
        idx = min(idx, len(seg_len) - 1)

        if seg_len[idx] < 1e-6:
            out.append(points[idx])
            continue

        ratio = (ss - s[idx]) / seg_len[idx]
        p = points[idx] * (1.0 - ratio) + points[idx + 1] * ratio
        out.append(p)

    return np.array(out, dtype=float), total_len


def smooth_closed(points, iters):
    pts = points.copy()
    for _ in range(iters):
        prev_pts = np.roll(pts, 1, axis=0)
        next_pts = np.roll(pts, -1, axis=0)
        pts = 0.25 * prev_pts + 0.50 * pts + 0.25 * next_pts
    return pts


def compute_heading(points):
    n = len(points)
    psi = np.zeros(n)

    for i in range(n):
        p_prev = points[(i - 1) % n]
        p_next = points[(i + 1) % n]
        dx = p_next[0] - p_prev[0]
        dy = p_next[1] - p_prev[1]
        psi[i] = math.atan2(dy, dx)

    return psi


def compute_curvature(points, ds):
    n = len(points)
    kappa = np.zeros(n)

    for i in range(n):
        p_prev = points[(i - 1) % n]
        p = points[i]
        p_next = points[(i + 1) % n]

        dx = (p_next[0] - p_prev[0]) / (2.0 * ds)
        dy = (p_next[1] - p_prev[1]) / (2.0 * ds)

        ddx = (p_next[0] - 2.0 * p[0] + p_prev[0]) / (ds * ds)
        ddy = (p_next[1] - 2.0 * p[1] + p_prev[1]) / (ds * ds)

        denom = (dx * dx + dy * dy) ** 1.5
        if denom > 1e-6:
            kappa[i] = (dx * ddy - dy * ddx) / denom
        else:
            kappa[i] = 0.0

    return kappa


def make_speed_profile(kappa, v_min, v_max, curv_gain):
    # 곡률이 큰 곳에서는 속도를 낮춤
    v = v_max / (1.0 + curv_gain * np.abs(kappa))
    v = np.clip(v, v_min, v_max)
    return v


def make_wpnt_array(points, psi, kappa, speed, ds):
    arr = WpntArray()
    arr.header.frame_id = "map"

    for i, (x, y) in enumerate(points):
        wp = Wpnt()
        wp.id = int(i)
        wp.s_m = float(i * ds)
        wp.d_m = 0.0
        wp.x_m = float(x)
        wp.y_m = float(y)

        # 수동 waypoint라 실제 좌우 track bound는 모르므로 보수적 임시값
        wp.d_right = 0.6
        wp.d_left = 0.6

        wp.psi_rad = float(psi[i])
        wp.kappa_radpm = float(kappa[i])
        wp.vx_mps = float(speed[i])
        wp.ax_mps2 = 0.0

        arr.wpnts.append(wp)

    return arr


def make_markers(points, psi, ns, color, scale=0.08):
    markers = MarkerArray()

    for i, (x, y) in enumerate(points):
        m = Marker()
        m.header.frame_id = "map"
        m.ns = ns
        m.id = int(i)
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.02

        q = quaternion_from_euler(0.0, 0.0, float(psi[i]))
        m.pose.orientation.x = float(q[0])
        m.pose.orientation.y = float(q[1])
        m.pose.orientation.z = float(q[2])
        m.pose.orientation.w = float(q[3])

        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0

        markers.markers.append(m)

    return markers


def make_line_marker(points):
    markers = MarkerArray()

    line = Marker()
    line.header.frame_id = "map"
    line.ns = "manual_centerline_line"
    line.id = 0
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.scale.x = 0.03
    line.color.r = 0.0
    line.color.g = 1.0
    line.color.b = 0.0
    line.color.a = 0.8

    for x, y in points:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        line.points.append(p)

    p0 = Point()
    p0.x = float(points[0, 0])
    p0.y = float(points[0, 1])
    p0.z = 0.0
    line.points.append(p0)

    markers.markers.append(line)
    return markers


def save_dense_csv(path, points, psi, kappa, speed):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "x", "y", "psi_rad", "kappa_radpm", "vx_mps"])
        for i, (p, yaw, curv, v) in enumerate(zip(points, psi, kappa, speed)):
            writer.writerow([i, p[0], p[1], yaw, curv, v])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-dir", required=True)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--ds", type=float, default=0.10)
    parser.add_argument("--v-min", type=float, default=0.25)
    parser.add_argument("--v-max", type=float, default=0.45)
    parser.add_argument("--curv-gain", type=float, default=2.0)
    parser.add_argument("--smooth-iters", type=int, default=1)
    args = parser.parse_args()

    rclpy.init(args=None)

    map_dir = Path(args.map_dir).expanduser()
    csv_path = Path(args.csv).expanduser()

    raw_points = read_points(csv_path)
    dense_points, total_len = resample_closed_path(raw_points, args.ds)

    if args.smooth_iters > 0:
        dense_points = smooth_closed(dense_points, args.smooth_iters)

    psi = compute_heading(dense_points)
    kappa = compute_curvature(dense_points, args.ds)
    speed = make_speed_profile(kappa, args.v_min, args.v_max, args.curv_gain)

    wpnts = make_wpnt_array(dense_points, psi, kappa, speed, args.ds)

    centerline_markers = make_markers(dense_points, psi, "manual_centerline", (1.0, 1.0, 0.0))
    global_markers = make_markers(dense_points, psi, "manual_global_waypoints", (0.0, 0.0, 1.0))
    sp_markers = make_markers(dense_points, psi, "manual_shortest_path", (1.0, 0.0, 0.0))
    trackbounds = make_line_marker(dense_points)

    map_info = String()
    map_info.data = (
        f"manual dense waypoints from {csv_path.name}, "
        f"raw={len(raw_points)}, dense={len(dense_points)}, ds={args.ds}"
    )

    est_lap_time = Float32()
    est_lap_time.data = float(total_len / max(float(np.mean(speed)), 0.1))

    write_global_waypoints(
        map_dir=str(map_dir),
        map_info_str=map_info.data,
        est_lap_time=est_lap_time,
        centerline_markers=centerline_markers,
        centerline_waypoints=deepcopy(wpnts),
        global_traj_markers_iqp=global_markers,
        global_traj_wpnts_iqp=deepcopy(wpnts),
        global_traj_markers_sp=sp_markers,
        global_traj_wpnts_sp=deepcopy(wpnts),
        trackbounds_markers=trackbounds,
    )

    save_dense_csv(map_dir / "manual_waypoints_dense.csv", dense_points, psi, kappa, speed)

    print(f"Created: {map_dir / 'global_waypoints.json'}")
    print(f"Created: {map_dir / 'manual_waypoints_dense.csv'}")
    print(f"Raw points: {len(raw_points)}")
    print(f"Dense points: {len(dense_points)}")
    print(f"Total length: {total_len:.2f} m")
    print(f"Speed min/max: {float(np.min(speed)):.2f} / {float(np.max(speed)):.2f} m/s")
    print(f"Mean |kappa|: {float(np.mean(np.abs(kappa))):.3f}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()