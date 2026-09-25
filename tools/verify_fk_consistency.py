#!/usr/bin/env python3
"""Compare URDF FK against the Isaac-measured gripper_base pose from Lesson 6.

FK itself now comes from research/src/mpg/urdf_fk.py (shared with
tools/send_joint_cmd.py and research/scripts/select_demo_poses.py) instead of
an inline recomputation -- see research/tests/test_urdf_fk.py for the
independent cross-check that this refactor didn't change the numbers.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research" / "src"))

from mpg.urdf_fk import UrdfChainFk  # noqa: E402


def quat_xyzw(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


urdf_path = ROOT / "ros2_ws/src/robot129_description/urdf/robot129.urdf"
chain = UrdfChainFk(urdf_path)

probe = json.loads((ROOT / "out/lesson_06/gripper_probe.json").read_text())
sample = probe["measurements"]["open"]
joint_values = dict(zip(probe["joint_names"], sample["joint_position"]))

T = chain.forward_kinematics(joint_values, "gripper_base")

isaac = sample["bodies"]["gripper_base"]
p = np.array(isaac["position"])
R = quat_xyzw(isaac["quaternion_xyzw"])
trans = float(np.linalg.norm(T[:3, 3] - p))
angle = float(math.acos(np.clip((np.trace(T[:3, :3].T @ R) - 1) / 2, -1, 1)))
checks = {"translation_under_1mm": trans < .001, "rotation_under_0_1deg": angle < math.radians(.1)}
report = {
    "status": "PASS" if all(checks.values()) else "FAIL",
    "link": "gripper_base",
    "joint_state_source": "out/lesson_06/gripper_probe.json",
    "urdf_position_m": T[:3, 3].tolist(),
    "isaac_position_m": p.tolist(),
    "translation_error_m": trans,
    "rotation_error_rad": angle,
    "rotation_error_deg": math.degrees(angle),
    "checks": checks,
}
out = ROOT / "out/lesson_05/fk_consistency.json"
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
raise SystemExit(0 if report["status"] == "PASS" else 1)
