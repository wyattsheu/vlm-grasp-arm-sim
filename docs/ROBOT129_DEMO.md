# Robot 129 模擬 Demo 操作手冊

Workspace：`/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913`

這套 Demo 使用官方 Piper 3D/URDF 衍生出的 Robot 129 description 與 USD。Isaac Sim 是現有的 6.0.1.0；沒有重裝或升級。所有 ROS 操作固定在 domain 129、namespace `/robot129_sim`，硬體 driver 數量為 0。

## 最快看成果：已完成影片

影片位於：

    out/final_demo/robot129_full_simulation.mp4

在 server 有桌面播放器時：

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    mpv out/final_demo/robot129_full_simulation.mp4

也可以把 MP4 下載到自己的電腦或直接用 VS Code/Codex 檔案瀏覽器開啟。接觸表在 `out/final_demo/contact_sheet.png`。

影片 19 秒，H.264、640×480、30 fps：

- 0–2 秒：版本與 simulation-only 範圍。
- 2–12 秒：真正 PhysX 雙指接觸、LIFT、藍色 WAYPOINT、綠色 RELEASE。紅色方塊沒有 pose attachment。
- 12–16 秒：Isaac RGB-D SceneBundle 的 GRASP／WAYPOINT／RELEASE grounding replay。
- 16–19 秒：ROS、MoveIt、MTC 與 physics 實測結果。

紅色 GRASP 表示目標抓取點；藍色 WAYPOINT 表示搬運途中必須經過的空間點；綠色 RELEASE 表示開爪放下的位置。這三點讓 perception 輸出可以轉成可檢查的任務階段，並讓錯誤能被歸因到 grounding、幾何、規劃、controller 或 physics。

## WebRTC 看即時 Robot 129

若要用 ROS 自己控制機器人，請使用 `bash tools/start_robot129_ros_webrtc.sh`，再從另一個終端機送 ROS 命令；完整流程見 `docs/LOCAL_ROS_CONTROL.md`。以下的 `start_robot129_webrtc.sh` 是會在 HOME 畫面後自行執行抓取與放下的 physics 驗收展示。

啟動器會選當下較低負載的 GPU，直接執行，不等待 GPU 空閒。啟動過程會依序顯示「載入 Isaac Sim」、「RTX／viewport 暖機」與「截圖驗證」；只有伺服器成功擷取真正的 1280×720 viewport、確認不是黑畫面／純色畫面，且紅色目標方塊大小足夠時，才回報 PASS。

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
bash tools/start_robot129_webrtc.sh
```

在 NVIDIA Isaac Sim WebRTC Streaming Client 輸入：

- Server IP：`140.113.203.85`
- Signaling port：`49100`
- Stream port：`47998`

若 client 還留著舊程序的連線，先按 Disconnect，再重新 Connect。串流固定為 1280×720、30 fps，不接受 client 動態改成 1920×1080。操作視角使用 `Alt + 左鍵` 旋轉、`Alt + 中鍵` 平移、滾輪縮放。

這台 server 的 IsaacLab headless loop 必須在每個物理 step 前額外執行兩次 Kit `app.update()`。這會持續交付 WebRTC frame 並處理遠端滑鼠事件；缺少它時，port 雖已開啟，畫面仍會卡住或變黑。直播直接控制 `/OmniverseKit_Persp`，沒有建立第二個 sensor render product，因此不會讓 Replicator 相機污染 viewport。Fabric render-delegate transform read 與 per-environment RTX scene partition 也維持關閉。

HOME 畫面保持 10 秒後執行 PhysX 抓取，完成後持續保留場景，直到執行 stop。伺服器會留下兩張驗收圖：

- `out/webrtc_robot129/viewport_probe.png`：動作前，需清楚看到 Robot 129 與紅色方塊。
- `out/webrtc_robot129/viewport_probe_after_demo.png`：完整動作後，用來排除幾秒後變黑／變灰。

查狀態與停止：

```bash
bash tools/status_robot129_webrtc.sh
bash tools/stop_robot129_webrtc.sh
```

若 49100 已被其他 Isaac stream 佔用，啟動器會停止並列出程序，不會誤把其他 Demo 的 port 當成成功。

## 階段 A：重跑真正 PhysX 抓取

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    bash tools/verify_robot129_physics_grasp.sh

這個 wrapper 先用 `nvidia-smi` 選較低負載 GPU，然後以既有 IsaacLab/Isaac Sim 啟動 Robot 129。它建立 0.08 kg dynamic cube、雙側 contact sensors，關爪後開啟重力，抬升、保持、放開，再把 300 張畫格編成影片。

預期最後 JSON 為 PASS，且同時滿足：雙側 contact > 0.1 N、抬升 > 0.03 m、放開後落下、沒有 kinematic attachment。產物在 `out/lesson_06/physics_grasp`。這個步驟會使用 GPU，通常約數十秒到數分鐘。

## 階段 B：驗證 ROS 隔離與控制

以下命令都只使用 domain 129；不需 GPU：

    bash tools/verify_ros_isolation.sh
    bash tools/verify_ros2_control_mock.sh
    bash tools/verify_ros2_control_guards.sh

第一個證明 domain 0 收不到 Robot 129 訊息。第二個啟用 simulation GenericSystem、joint-state broadcaster、arm/gripper trajectory controllers。第三個發送正常 goal、以 watchdog 取消長 goal，再送 recovery goal。

ROS 指令真正進入 Isaac articulation 的 roundtrip 需要 GPU：

    bash tools/verify_robot129_ros_roundtrip.sh

預期 report 為 PASS、domain 0 messages 為 0、最大終點誤差小於 0.01 rad/m。

## 階段 C：MoveIt 與 MTC

    bash tools/verify_moveit_fixed_plan.sh
    bash tools/verify_robot129_mtc.sh

第一個啟動 `move_group`，在 PlanningScene 放入障礙物，以 OMPL RRTConnect 規劃固定目標。第二個建立 CURRENT_STATE → APPROACH → GRASP → LIFT → TRANSIT → PREPLACE → RELEASE → RETREAT 的 MTC graph。兩者都只 plan，不送硬體命令。

## 階段 D：SceneBundle 與 MPG replay

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/research
    source /mnt/HDD4/wyattsheu/env_robot129_research/bin/activate
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python scripts/validate_scene.py ../out/integrated_demo/scene_bundle
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python scripts/validate_robot129_sim_pipeline.py

這裡不啟動 Isaac；它讀取已同步保存的 RGB、depth、CameraInfo、TF 與 JointState，重播三個 stage points 的 deprojection 與 world transform。overlay 路徑可由 `research/out/robot129_sim_replay/latest.json` 找到。

## 階段 E：總驗收

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    python3 tools/verify_completed_system.py

它讀取每一課的 PASS JSON、確認 URDF/USD/SceneBundle/影片非空，並寫出 `out/final_acceptance.json`。它不會重新跑 GPU 工作。

## 從來源重建的主要命令

    python3 tools/build_robot129_description.py
    bash tools/import_robot129_usd.sh

第一個只讀 vendor 並重建可修改的 `robot129_description`；第二個用固定 importer settings 重建 USD。不要修改 `robot/vendor`。

## 明確沒有完成的外部項目

- 上層 vLLM/VLM：獨立 vLLM 0.29.0 環境與 Qwen3-VL-8B-Instruct cache 已建立，smoke test PASS，服務預設停止；完整 live SceneBundle grounding 仍未驗證。
- Thor-connected：沒有 Thor host/隔離網路資料；只完成 server-local UDP adapter loopback。
- sim-to-real calibration：真實 camera intrinsics/extrinsics、controller gains、摩擦與 hardware revision 未驗證。
- 真實機器人：CAN、Piper SDK、RealSense USB 與 production ROS domain 全部未連接。
