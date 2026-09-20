# Robot 129 server-side simulation 完成報告

日期：2026-09-15（Asia/Taipei）

## 已建立

- 官方 Piper 3D/URDF 衍生的 `robot129_description`，vendor 保持唯讀。
- Isaac Sim 6.0.1.0 可重建 USD 與 8-joint articulation。
- 真正 PhysX contact grasp、lift、hold、waypoint、release、settle。
- 同步 RGB-D／CameraInfo／TF／JointState SceneBundle。
- domain 129、namespace `/robot129_sim` 的 ROS-to-Isaac roundtrip。
- simulation-only ros2_control arm/gripper controllers，以及 success/cancel/watchdog/recovery tests。
- MoveIt 固定目標碰撞規劃與 8-stage MoveIt Task Constructor graph。
- 實際 Isaac SceneBundle 的 deterministic MPG geometry replay。
- Thor deployment adapter 的 local loopback 與 stale/domain/backend guards。
- Robot 129 WebRTC server-ready demo 與 19 秒最終 H.264 影片。

## 實際執行的成功命令

    python3 tools/build_robot129_description.py
    bash tools/import_robot129_usd.sh
    python3 tools/verify_fk_consistency.py
    bash tools/verify_robot129_physics_grasp.sh
    bash tools/verify_ros_isolation.sh
    bash tools/verify_robot129_ros_roundtrip.sh
    bash tools/verify_ros2_control_mock.sh
    bash tools/verify_ros2_control_guards.sh
    bash tools/verify_moveit_fixed_plan.sh
    bash tools/verify_robot129_mtc.sh
    /mnt/HDD4/wyattsheu/env_robot129_research/bin/python research/scripts/validate_robot129_sim_pipeline.py
    /mnt/HDD4/wyattsheu/env_robot129_research/bin/python deployment/verify_thor_loopback.py
    python3 tools/build_final_demo_video.py
    python3 tools/verify_completed_system.py
    bash tools/start_robot129_webrtc.sh
    bash tools/stop_robot129_webrtc.sh

ROS workspace 的主要 build：

    /mnt/HDD4/wyattsheu/tools/micromamba/micromamba run \
      -p /mnt/HDD4/wyattsheu/env_robot129_ros bash -lc \
      'source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash && cd ros2_ws && colcon build --symlink-install'

## Bundle 驗證語意

`tools/verify_bundle.sh` 是「尚未 build 的 pristine handoff」檢查，Lesson 1 執行時 PASS。colcon `--symlink-install` 之後再跑會預期回報 bundle 含 symlink；這些都在 `ros2_ws/build`／`install`。最終改跑 `tools/verify_sources_post_build.sh`，`provenance/SHA256SUMS` 全部 OK，證明 vendor、reference 與原始 handoff 檔未被修改。

## 驗收摘要

- URDF：17 links、16 joints、20 mesh references，PASS。
- USD：69 prims、6 revolute、2 prismatic，PASS。
- FK：`1.58e-7 m`、`0 rad`，PASS。
- PhysX：雙指約 `10.06 N`；payload `0.450 → 0.606 m`，PASS。
- ROS-to-Isaac：402 joint-state messages；domain 0 為 0；誤差 `0.0069567`，PASS。
- ros2_control：3 controllers active；normal/cancel/recovery 全 PASS。
- MoveIt：RRTConnect 21 points，終點誤差 `0.000749 rad`，PASS。
- MTC：8 stages、1 solution，PASS。
- MPG replay：3/3 points 取得 camera/world coordinates，PASS。
- Thor loopback：20 observations，stale/duplicate/hardware/domain guards 全 PASS。
- WebRTC：Robot 129 server READY、49100 listening，PASS；外部 client session NOT RUN。
- 總驗收：`out/final_acceptance.json` PASS，硬體命令 0。

## 影片

- `out/final_demo/robot129_full_simulation.mp4`
- H.264、640×480、30 fps、570 frames、19 秒。
- SHA-256：`f7f663783f5b69de5ad2c2e5284a81c86e8b48d39797bcfd55201c17731d40d2`

## NOT RUN／UNVERIFIED

live VLM 與 Thor-connected rehearsal 需要目前不存在的模型／套件或外部 Thor 網路資訊。這兩項沒有被包裝成假的 PASS。真實 camera calibration、hardware revision、contact/controller 實測參數與所有 real-robot 行為也維持 UNVERIFIED。
