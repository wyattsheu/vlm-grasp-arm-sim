# Lesson 2 — 用官方 Cartpole 理解 articulation

日期：2026-09-14  
狀態：PASS

## 這一課在建立什麼

這一課先使用 Isaac Lab 內建的官方 Cartpole，而不是直接處理 Piper。Cartpole 只有兩個 joint，可以清楚看出以下結構：

- Stage：整個模擬世界。
- Prim：Stage 裡有路徑的物件，例如 `/World/Robot`。
- Rigid body：會受物理影響的單一剛體。
- Joint：限制兩個剛體之間如何運動。
- Articulation：由多個 rigid bodies 和 joints 組成、由物理引擎一起求解的機構。
- Simulation step：讓物理時間前進一格，本課為 1/60 秒。
- Reset：把 root pose、速度與 joint state 恢復到預設狀態。

兩個 joints 是：

- `slider_to_cart`：小車沿軌道平移，位置單位為公尺。
- `cart_to_pole`：長桿繞小車旋轉，位置單位為弧度。

Piper 六軸手臂使用相同觀念，只是 link、joint、limit 與 drive 更多。

## 架構位置

    Isaac Sim Stage
    └── /World
        ├── defaultGroundPlane
        ├── Light
        ├── LessonCamera
        └── Robot                 ← articulation root prim
            ├── slider            ← rigid body／軌道
            ├── cart              ← rigid body／小車
            ├── pole              ← rigid body／長桿
            ├── slider_to_cart    ← prismatic joint
            └── cart_to_pole      ← revolute joint

## 已建立的檔案

- 官方參考：`/mnt/HDD4/wyattsheu/IsaacLab/scripts/tutorials/01_assets/run_articulation.py`
- 教學程式：`lessons/lesson_02/cartpole_articulation.py`
- 一鍵入口：`tools/run_lesson_02.sh`
- 最終產物：`out/lesson_02/20260913T235241Z/`

教學程式沿用目前 Isaac Lab 3.0 的 `CARTPOLE_CFG` 和 `Articulation` API。它執行固定 240 steps，對平移 joint 施加可重現的正弦 effort，記錄 reset 前後位置、Stage prim paths，並使用 Isaac Camera sensor 保存 RGB 圖片。

它不啟動 ROS、不連接硬體，也不使用 Piper SDK。

## 你之後重跑整階段的命令

一次執行整段即可，不需要逐行回傳：

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    nvidia-smi --query-gpu=index,memory.used,memory.total,utilization.gpu --format=csv
    bash tools/run_lesson_02.sh
    run_dir="$(cat out/lesson_02/latest_run.txt)"
    cat "$run_dir/articulation_observation.json"
    ls -lh "$run_dir/cartpole.png"

入口會選兩張卡中綜合利用率與 VRAM 較低的一張並直接執行，不再等待 GPU 低於固定門檻。若本帳號已有另一個 Isaac Python process，仍會停止，避免重複啟動。`uv run --no-sync` 不會更新或重裝 Isaac 環境。

## 執行後會看到什麼

終端的關鍵行為：

    [LESSON 2] articulation prim: /World/Robot
    [LESSON 2] joint names: ['slider_to_cart', 'cart_to_pole']
    [LESSON 2] before reset: [0.3714..., 4.5434...]
    [LESSON 2] after reset: [-0.00038..., -0.00059...]
    [LESSON 2] result: PASS
    PASS: Lesson 2 runtime completed.

`cartpole.png` 會看到淡黃色水平軌道、淡藍色小車及紫灰色長桿。畫面證明 USD prim 被建立和渲染；JSON 裡的數字才證明 joint 可被施力、physics 有前進、reset 有作用。

## 驗收結果

- articulation prim：`/World/Robot`
- joint names：`slider_to_cart`、`cart_to_pole`
- physics dt：0.0166667 秒
- reset 前：`[0.371425, 4.543474]`
- reset 後：`[-0.000387, -0.000597]`
- reset 最大絕對誤差：0.000597，小於門檻 0.05
- Camera RGB：640 × 480 PNG，已目視確認模型、軌道和地面可辨識
- 最終 status：PASS

## Done

- 以目前 checkout 自帶的官方 Cartpole asset 完成 articulation walkthrough。
- 建立固定步數、會自動退出的一鍵教學程式。
- 建立 console、JSON 及 Camera RGB 證據。
- 修正非同步 viewport 空白／延後寫檔問題，改用 Camera sensor 的明確 RGB buffer。
- GPU 規則依使用者需求改為自動選較低負載卡並直接執行。
- 建立可持續運行的 WebRTC 上課 Demo，以及 start／status／stop 工具。

## Verified

- Isaac Sim 6.0.1.0 與 Isaac Lab 3.0.0 runtime：PASS。
- PhysX articulation 建立及 240 steps：PASS。
- joint effort 造成狀態改變：PASS。
- reset 誤差驗收：PASS。
- Stage prim tree：PASS。
- 本地 RGB 圖片：PASS，已目視檢查。
- 外層 shell 會檢查 Python 退出碼、JSON status 及非空 PNG。
- WebRTC extensions、Isaac app ready、Demo READY 與 TCP 49100 listening：PASS。

## NOT RUN／UNVERIFIED

- Piper URDF、mesh、joint 與 TF：本課不執行，留到 Lesson 3。
- ROS 2、MoveIt、ros2_control：本課不執行。
- 真實硬體、CAN、Piper SDK、RealSense、production ROS domain：未連接。
- GPU 壅塞時的 wall-clock 效能可能變慢；本課不把執行時間當物理正確性的驗收條件。
- 外部 WebRTC Client 的端到端連線：等待使用者在 Client 按 Connect 確認。

## 產物路徑

- `out/lesson_02/20260913T235241Z/console.log`
- `out/lesson_02/20260913T235241Z/articulation_observation.json`
- `out/lesson_02/20260913T235241Z/cartpole.png`
- `out/lesson_02/latest_run.txt`
- `docs/lesson_02_webrtc_demo.md`
- `lessons/lesson_02/cartpole_webrtc_demo.py`
- `tools/start_lesson_02_webrtc.sh`
- `tools/status_lesson_02_webrtc.sh`
- `tools/stop_lesson_02_webrtc.sh`

## 下一課

Lesson 3 將只讀檢查 Piper 官方來源副本：展開 URDF／Xacro，核對 link、joint、limit、mesh 路徑與 TF tree。任何修正不會寫入 `robot/vendor`。依課程規則，Lesson 2 到此停止，等使用者確認後再開始。
