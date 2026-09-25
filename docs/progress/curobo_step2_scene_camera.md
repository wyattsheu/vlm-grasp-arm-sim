# cuRobo Step 2 — 後方立柱相機（`--scene-camera pole`）

狀態：PASS（程式改好、真的用 headless Isaac 啟動驗證過，不只是程式碼審查）；`pick_place_counter`／更遠場景視野 NOT RUN

完整指令、座標、實測數字見 `docs/dev_guide_paper_core_and_dashboard_plan.md` §7.4。

## 概念與架構

使用者指示：只模擬相機本身和它站的細柱，不模擬車台。相機座標（`research/configs/scene_camera.yaml`，status `ROUGH_FROM_PHOTO`）是從使用者提供的照片＋AR 測距草圖估的，使用者確認「大致對，先用推估值」，不是量測值。`sim/scripts/run_robot129_ros_webrtc.py` 新增 `--scene-camera {off,pole}`（預設 off，行為完全不變），把手腕相機原本的 publish/save 邏輯重構成共用函式，新相機用同一套邏輯，換掉 frame_id 和 topic 前綴。

## Done／Verified

- 用 `tools/start_robot129_grasp_sim.sh --scene pick_place --scene-camera pole` 真的啟動過 Isaac，`READY`，log 沒有新錯誤。
- ROS 端確認：`scene_camera/{color,aligned_depth_to_color}/...` 三個 topic 都有資料在發布（~1.85Hz RGB、~3.4Hz depth，這台機器目前 GPU 被佔滿只有約 0.16x 即時速度）。
- `tf.json`：位置精確等於設定值；四元數 `(-0.5, 0.5, -0.5, 0.5)`跟手算「水平看 +X、up=+Z、ROS optical 慣例」完全一致（獨立驗證兩次，一次手算矩陣轉四元數，一次讀 Isaac 實際輸出）。
- `depth.npy`：87957/307200 像素有限值，範圍 0.24–2.15m，合理。`camera_info.json`：fx=fy=855.17px，跟手算值一致。
- 迴歸測試：不加 `--scene-camera`（預設 off）重新啟動過一次，`scene_camera` topic 數量 = 0，手腕相機照常運作，log 沒有新錯誤——確認新功能是真的 opt-in，沒有動到原本行為。
- 錄影新增第三個視角（`scene_frames/` → `scene_video.mp4`），只在 `--scene-camera pole` 時才建立。**真的觸發過一次錄影**（不只是看程式碼）：`ros2 service call /robot129_sim/recording std_srvs/srv/SetBool "{data: true}"` → 等待幾秒 → `{data: false}`，產出 `out/grasp_motion/sessions/run_0010/scene_video.mp4`（15 frames，跟 `video.mp4`/`wrist_video.mp4` 一起），server log 印出 `RECORDING_STOP ... scene_frames=15 -> .../scene_video.mp4`。

## NOT RUN／UNVERIFIED

- HOME 姿態下畫面主要是手臂自己的夾爪和一小條地板（已預期，見 dev guide 的已知限制段落）——`pick_place_counter`（0.25m 高台面）或其他場景下這顆相機實際看得到多少東西，還沒測。
- 相機座標本身沒有校準過，等使用者提供量測數據再更新 `scene_camera.yaml`。
- 手腕相機碰撞球（Step 1 就跳過的）跟這顆新相機的碰撞幾何都還沒做，`scene_camera.yaml` 的 `pole` 欄位目前只有柱子本身的碰撞體，沒有相機外殼。
