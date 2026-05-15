#!/usr/bin/env python3
import json
from pathlib import Path

MAP = "25_real_track"
map_dir = Path.home() / "F1_TENTH_UNITA/src/race_stack/stack_master/maps" / MAP
json_path = map_dir / "global_waypoints.json"

if not json_path.exists():
    raise FileNotFoundError(f"global_waypoints.json not found: {json_path}")

with open(json_path, "r") as f:
    data = json.load(f)

wpnts = data["global_traj_wpnts_iqp"]["wpnts"]
n = len(wpnts)

if n < 2:
    raise RuntimeError(f"Waypoint count too small: {n}")

end_idx = n - 1

speed_scaling_yaml = f"""sector_tuner:
  ros__parameters:
    global_limit: 0.35
    n_sectors: 1
    Sector0:
      start: 0
      end: {end_idx}
      scaling: 0.35
      only_FTG: false
      no_FTG: false
"""

ot_sectors_yaml = f"""ot_interpolator:
  ros__parameters:
    n_sectors: 1
    yeet_factor: 1.25
    spline_len: 30
    ot_sector_begin: 0.5
    Overtaking_sector0:
      start: 0
      end: {end_idx}
      ot_flag: false
"""

(map_dir / "speed_scaling.yaml").write_text(speed_scaling_yaml)
(map_dir / "ot_sectors.yaml").write_text(ot_sectors_yaml)

print(f"Waypoint count: {n}")
print(f"Sector end index: {end_idx}")
print("Created speed_scaling.yaml and ot_sectors.yaml")
