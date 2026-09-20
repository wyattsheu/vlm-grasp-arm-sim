# Robot 129 development handoff completion

Date: 2026-09-15
Scope: development manual, external-system data contracts, calibration gates, and controller mapping correction.

## Done

- Added `docs/DEVELOPMENT_MANUAL.md` with the three-system architecture, complete workspace map, environment entry points, edit/build/test commands, data flow, WebRTC demo, and daily workflow.
- Documented live vLLM/VLM requirements and historical Qwen/Molmo baseline; a separate PRO 6000 vLLM environment and Qwen3-VL-8B-Instruct cache were later installed and smoke-tested.
- Documented Thor-connected rehearsal and added an approval/readiness contract for a simulation-only test network.
- Added active data contracts for camera intrinsics, hand-eye extrinsics, camera registration, hardware revision, physical measurements, gripper mapping, real controller characterization, and contact characterization.
- Added a read-only original-system inventory collector that does not start/stop containers, join ROS by default, inspect secret environment values, or touch hardware.
- Added readiness gates that reject missing files, template statuses, blank required fields, and legacy calibration candidates.
- Added numerical camera validation for K, depth scale, aligned pixel grid, homogeneous transform, SO(3), frame direction, and validation metrics.
- Corrected MoveIt `gripper_controller` to command only leader `joint7`; mimic follower `joint8` remains in robot geometry/planning.
- Made the MoveIt fixed-plan wrapper terminate its background `move_group` reliably.

## Commands executed

```bash
/mnt/HDD4/wyattsheu/tools/micromamba/micromamba run \
  -p /mnt/HDD4/wyattsheu/env_robot129_ros bash -lc \
  'source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash; \
   cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/ros2_ws; \
   colcon build --symlink-install --packages-select robot129_moveit_config'

timeout 90s bash tools/verify_moveit_fixed_plan.sh
timeout 120s bash tools/verify_robot129_mtc.sh
bash tools/validate_camera_calibration.sh --self-test
bash tools/check_robot129_readiness.sh --require simulation

source /mnt/HDD4/wyattsheu/env_robot129_research/bin/activate
cd research
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python -m unittest discover -s tests -v
cd ..
bash tools/verify_sources_post_build.sh
python3 tools/verify_completed_system.py
```

## Verified

- MoveIt fixed planning: PASS, 22 trajectory points, final goal error 0.000495424 rad, hardware commands 0.
- MTC eight-stage planning: PASS, one solution, hardware commands 0.
- Camera validator self-test: valid fixture PASS; invalid non-SO(3) fixture FAIL as required.
- Research unit tests: 108 PASS.
- Handoff source checksums: PASS; original templates and vendor files unchanged.
- Full server-side simulation acceptance: PASS.
- Simulation readiness: PASS.

## NOT RUN / UNVERIFIED

- PRO 6000 upper-level vLLM smoke test is PASS and stopped after use. Original Robot 129/Thor container digests, exact legacy revisions and live SceneBundle grounding remain UNVERIFIED.
- Thor-connected rehearsal: Thor IP, isolated test network, DDS config, clock budget, and approved launch/rollback commands are missing. Local adapter loopback alone is PASS.
- Real camera: no physical CameraInfo capture, board calibration, hand-eye dataset, reprojection validation, or device revision was performed.
- Real controller/contact: no supervised low-speed robot trial, contact-signal trial, or hardware revision evidence was collected.
- CAN, Piper SDK/driver, RealSense USB, and production ROS domain were not connected.

## Main artifacts

- `docs/DEVELOPMENT_MANUAL.md`
- `environment/vlm_source_inventory.template.yaml`
- `deployment/thor_rehearsal.template.yaml`
- `calibration/registry.yaml`
- `calibration/*.active.template.yaml`
- `tools/collect_original_system_readonly.sh`
- `tools/check_robot129_readiness.sh`
- `tools/validate_camera_calibration.sh`
- `out/readiness/development_readiness.json`
- `out/final_acceptance.json`
- `out/final_demo/robot129_full_simulation.mp4`
