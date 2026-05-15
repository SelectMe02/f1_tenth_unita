#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class ClickWaypointRecorder(Node):
    def __init__(self, out_csv: Path):
        super().__init__("click_waypoint_recorder")
        self.out_csv = out_csv
        self.points = []

        self.sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_cb,
            10
        )

        self.get_logger().info("RViz의 Publish Point 버튼으로 waypoint를 순서대로 찍어줘.")
        self.get_logger().info("다 찍었으면 Ctrl+C. 저장 파일: " + str(self.out_csv))

    def clicked_cb(self, msg: PointStamped):
        x = float(msg.point.x)
        y = float(msg.point.y)
        self.points.append((x, y))
        self.get_logger().info(f"[{len(self.points)-1}] x={x:.4f}, y={y:.4f}")

    def save(self):
        self.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with open(self.out_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            for x, y in self.points:
                writer.writerow([x, y])
        self.get_logger().info(f"Saved {len(self.points)} points to {self.out_csv}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = ClickWaypointRecorder(Path(args.out).expanduser())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
