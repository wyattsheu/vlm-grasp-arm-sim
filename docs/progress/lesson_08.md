# Lesson 8 — ROS 2 Bridge 與 ros2_control

狀態：PASS（simulation-only）

## 概念與架構

ROS domain 隔離 DDS discovery，namespace 隔離 topic/action 名稱。`ros2_control` 將 controller 與硬體介面分開；本課的硬體介面固定為 `mock_components/GenericSystem` 或 Isaac articulation，絕不載入 Piper/CAN driver。

## Done／Verified

- user-space ROS 2 Jazzy、ros2_control 4.47.0、controllers 4.42.1 已建立於 `/mnt/HDD4/wyattsheu/env_robot129_ros`。
- domain 129 接收 81 則 `/robot129_sim/joint_states`；domain 0 接收 0，`hardware_drivers=0`。
- ROS trajectory 寫入真正 Isaac 8-joint articulation，再由 ROS joint states 回傳 402 則；domain 0 為 0；最大最終誤差 `0.0069567 rad/m`。
- simulation controller manager 啟用 joint-state broadcaster、六軸 arm controller 與單主動關節 gripper controller。
- action guard 驗收：正常 goal 成功；8 秒 goal 在 0.25 秒 watchdog 後取消；回報 CANCELED；接著 recovery goal 成功。
- 報告：`out/lesson_08/ros_isolation.json`、`ros_isaac_roundtrip.json`、`ros2_control_mock.json`、`ros2_control_guards.json`。

## NOT RUN／UNVERIFIED

- 未啟動 production ROS domain、CAN、Piper SDK 或任何硬體 driver。
- Isaac ROS Bridge extension 的跨程序 topic graph 尚未作為 production deployment 測試；目前 roundtrip 使用 Isaac 內建 Jazzy rclpy 與同一模擬程序。

## 下一課

以 MoveIt 固定目標與 MTC stage graph 驗證碰撞規劃。
