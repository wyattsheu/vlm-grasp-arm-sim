# Instructions for the Robot 129 simulation workspace

This workspace is a simulation and learning environment. Communicate with Wyatt in Traditional Chinese unless he requests another language.

## Teaching workflow

Follow `LEARNING_PATH.md` in order. Work on one lesson at a time. At the start of each lesson explain:

1. what component is being built;
2. why it exists in a robot system;
3. which files and commands will be used;
4. what observable result proves the lesson passed.

Then implement the lesson with Wyatt, inspect the output, and write `docs/progress/lesson_NN.md`. Do not claim success unless the command was run and the named artifact exists. Mark unavailable work `NOT RUN` and uncertain claims `UNVERIFIED`.

Do not skip directly to the final pick-and-place application. Use the official example robot first, then Piper asset validation, sensors, ROS control, MoveIt, and finally MPG.

## Workspace and reproducibility

- First run `tools/verify_bundle.sh` and `tools/collect_server_info.sh`.
- Read `README.md`, `docs/pro6000_sim_migration_plan.md`, and `provenance/README.md` before changing files.
- Keep source, generated assets, caches, run outputs, and third-party files in separate directories.
- Treat `robot/vendor/` as read-only. Create corrected descriptions under `ros2_ws/src/robot129_description/` and record every deviation.
- Treat `robot/reference_moveit_config/` as reference-only until each joint, frame, controller, and limit is verified.
- Pin Isaac Sim, ROS, container, Python, and model versions in `environment/version_matrix.yaml`.
- Never copy `.venv`, ROS `build/install/log`, container layers, or x86 binaries to Thor.
- Never print, log, copy, or commit API keys or authentication files.
- Preserve third-party licenses and record source hashes.

## Simulation and hardware boundary

- This bundle contains no authorization to command the physical Piper, gripper, or Kachaka.
- Do not attach CAN, RealSense USB, Piper SDK, or production hardware containers to the simulation workspace.
- Before enabling simulated motion, verify the chosen ROS domain and namespace are isolated from Robot 129 and record the evidence in the lesson report.
- Use explicit `simulation` and `hardware` backends. Never select hardware from topic discovery or an implicit fallback.
- Do not copy or launch the production `mm_actions_node` as a controller. Reuse its perception behavior only through documented adapters.
- Use standard state and command semantics: JointState for state; FollowJointTrajectory or a standard gripper controller action for commands.
- A successful trajectory is not a successful pick. Validate object lift, hold, transport, release, and settled pose.

## Robot model rules

- Confirm the exact Piper and gripper revision before treating the vendor URDF as authoritative.
- Maintain the ROS kinematic description as source material and generate USD reproducibly. Put Isaac-specific physics and sensor changes in explicit USD layers or versioned configuration.
- Keep visual and collision geometry separate. Include the wrist camera, mount, gripper, and carried payload in collision reasoning.
- Never guess TCP, camera transform, gripper width conversion, joint offsets, acceleration limits, mass, friction, or controller gains. Use typed placeholders until measured or sourced.
- Validate coordinate convention, transform direction, quaternion order, depth definition, units, and timestamps with synthetic known-geometry tests.

## Changes requiring a checkpoint with Wyatt

Pause after each lesson acceptance report. Also pause before system package installation, driver changes, Docker daemon changes, downloading large model or asset packs, opening external network services, connecting to Thor, or enabling any hardware backend. Present the exact commands, disk/network impact, and rollback before asking.

