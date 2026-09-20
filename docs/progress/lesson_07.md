# Lesson 7 — RGB-D、CameraInfo、TF 與 SceneBundle

狀態：PASS（Isaac recording camera；真實 wrist 外參 UNVERIFIED）

## 概念與架構

SceneBundle 將同一時間點的 RGB、metric depth、相機內參、相機到 world 的 transform、joint state 與指令封成可重播輸入，讓 vision 與 lifting 不必依賴正在執行的模擬器。

## Done／Verified

- Isaac Camera 同步產生 640×480 RGB 與 float32 optical-axis metric depth。
- CameraInfo 直接來自 `camera.data.intrinsic_matrices`，不是手填猜值。
- TF 由 `CameraData.pos_w + quat_w_ros` 建立，狀態 `AVAILABLE`；rotation orthogonality error `9.4e-7`，determinant `1.0000007`。
- SceneBundle 有效深度比例 `0.7770833`，hash 與 schema 驗證 PASS。
- 產物：`out/integrated_demo/scene_bundle`。
- MPG replay 再次驗證 `scene_bundle_valid=true` 與 `tf_available=true`。

## NOT RUN／UNVERIFIED

- 目前 frame 是 `recording_camera_optical_frame`，不是已標定的真實腕上 RealSense。
- 真實 camera intrinsics、畸變與 gripper-to-camera extrinsic 未提供。

## 下一課

在 domain 129 與 `/robot129_sim` 建立隔離的 ROS 2 控制契約。
