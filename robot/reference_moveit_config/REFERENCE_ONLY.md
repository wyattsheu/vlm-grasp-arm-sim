# Reference only

These files were copied from the existing `piper_with_gripper_moveit/config` directory for comparison. They have not been validated against Isaac Sim or the exact Robot 129 hardware revision.

Known review points:

- `piper.ros2_control.xacro` selects `mock_components/GenericSystem`.
- `ros2_controllers.yaml` commands joint1–joint6 and only joint7 for the gripper.
- `piper.srdf` uses link6 as the arm tip.
- acceleration limits are disabled in `joint_limits.yaml`.
- controller rates and gains must be selected against the simulation physics rate and measured tracking behavior.

Do not launch or copy these files into a final config wholesale. Build a new `robot129_moveit_config` during Lessons 8–9 and cite each reused value.

