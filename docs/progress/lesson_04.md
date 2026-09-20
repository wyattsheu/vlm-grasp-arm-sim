# Lesson 4 — robot129_description

狀態：PASS（nominal tool/camera frames 僅供模擬）

## 概念與架構

`robot/vendor` 是來源副本；可修改的 Robot 129 描述在 `ros2_ws/src/robot129_description`。產生器會複製有 provenance 的 mesh、加入 Robot 129 frame 與 simulation-only `ros2_control` 區塊。

## Done／Verified

- 建立可重現的 `robot129_description` package，17 links、16 joints、唯一 root `world`。
- 保留 fixed joints；加入 `flange`、`tool0`、`tcp`、`camera_mount`、`camera_link`、`camera_color_optical_frame`。
- `joint8` 以 multiplier `-1` mimic `joint7`；在 ros2_control 中只有主動的 `joint7` 可接受 command。
- `mock_components/GenericSystem` 明確標為 simulation-only；package 由 colcon build 通過。
- 產生指令：`python3 tools/build_robot129_description.py`。

## NOT RUN／UNVERIFIED

- TCP 與 wrist camera 外參尚未由真實標定確認。

## 下一課

以固定 importer 設定產生 USD 並比對 FK。
