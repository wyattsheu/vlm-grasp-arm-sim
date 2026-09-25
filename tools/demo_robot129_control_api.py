"""Demo of the simple Robot 129 control interface (robot129_control_api).

Run with the simulator already started (bash tools/start_robot129_ros_webrtc.sh):

    bash tools/run_with_robot129_control.sh tools/demo_robot129_control_api.py

or just: bash tools/run_robot129_control_demo.sh   (starts the sim too, if needed)

The two joint poses below are shaped like the requested example
(set_joint_positions -> close_gripper -> set_joint_positions -> open_gripper)
but use values that respect the real Piper joint limits in
robot/arm_parameter_summary.yaml (joint2 is limited to [0.0, 3.14] and joint3
to [-2.967, 0.0], not symmetric around zero).
"""

import json
from datetime import datetime, timezone
from pathlib import Path

from robot129_control_api import close_gripper, open_gripper, set_joint_positions

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out/ros_webrtc_robot129/control_api_demo.json"

POSE_A = [1.0, 2.20, -1.25, 0.0, 1.15, 0.0]
POSE_B = [-1.2, 1.35, -1.55, 0.3, -0.20, 0.10]


def main() -> int:
    steps = []

    print(f"[1/4] set_joint_positions({POSE_A})")
    steps.append({"call": "set_joint_positions", "args": POSE_A, "report": set_joint_positions(POSE_A)})

    print("[2/4] close_gripper()")
    steps.append({"call": "close_gripper", "args": [], "report": close_gripper()})

    print(f"[3/4] set_joint_positions({POSE_B})")
    steps.append({"call": "set_joint_positions", "args": POSE_B, "report": set_joint_positions(POSE_B)})

    print("[4/4] open_gripper()")
    steps.append({"call": "open_gripper", "args": [], "report": open_gripper()})

    overall = "PASS" if all(step["report"]["status"] == "PASS" for step in steps) else "FAIL"
    report = {
        "status": overall,
        "scope": "ROBOT129_CONTROL_API_DEMO",
        "simulation_only": True,
        "ros_domain_id": 129,
        "namespace": "/robot129_sim",
        "steps": steps,
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": overall, "report": str(OUT)}, indent=2))
    return 0 if overall == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
