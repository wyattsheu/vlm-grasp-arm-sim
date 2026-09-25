# Robot 129 模擬測試接口（給學長）

這是一個最小接口，把你的算法輸出接到 Isaac Sim 裡的 Robot 129 虛擬手臂，
可以直接用 WebRTC 看到動作結果。你不需要知道 ROS 或 Isaac Sim 的任何細節。

## 1. 你只需要用這三個 function

```python
from robot129_control_api import set_joint_positions, open_gripper, close_gripper

set_joint_positions([j1, j2, j3, j4, j5, j6])   # 六軸目標角度，單位 radian
close_gripper()                                  # 夾爪全關
open_gripper()                                   # 夾爪全開
```

呼叫後會**卡住等到手臂真的到位**（讀 Isaac 的 joint feedback 確認），
回傳一個 dict，例如 `{"status": "PASS", "target": [...], "feedback": [...], "max_error": 0.01}`。
`status` 是 `"PASS"` 代表確定有到位；`"FAIL"` 代表沒到位（通常是超出時間，可以重試或加長 `duration`）。

如果不想等待，加 `wait=False`：

```python
set_joint_positions([...], wait=False)   # 送出後立刻回傳，不等收斂
```

還有一個進階版本，如果你想要用數值（而不是 open/close）控制夾爪開合程度：

```python
from robot129_control_api import set_gripper
set_gripper(0.02)   # 0.0 = 全關，0.035 = 全開，單位公尺
```

### 六軸角度的合法範圍（超出會直接丟 `ValueError`，不會送進模擬器）

| joint | 範圍 (radian) |
|---|---|
| joint1 | -2.618 ~ 2.618 |
| joint2 | 0.0 ~ 3.14 |
| joint3 | -2.967 ~ 0.0 |
| joint4 | -1.745 ~ 1.745 |
| joint5 | -1.22 ~ 1.22 |
| joint6 | -2.0944 ~ 2.0944 |

注意 joint2、joint3 不是對稱區間（這是 Piper 手臂真實的關節限位，不是 bug）。

## 2. 怎麼跑你的程式

**第一步**：確認模擬器有在跑（如果還沒啟動，見下方「啟動模擬環境」）。

**第二步**：用這個 wrapper 執行你的程式，它會自動設好 ROS 環境，你的程式裡直接
`import robot129_control_api` 就能用：

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
bash tools/run_with_robot129_control.sh 你的程式.py
```

你完全不用自己 source ROS、設 `ROS_DOMAIN_ID`，這個 wrapper 都處理好了。

## 3. 啟動模擬環境（Isaac Sim + ROS + WebRTC 一次到位）

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
bash tools/start_robot129_ros_webrtc.sh
```

啟動完成會印出 `PASS: ROS-controlled Robot 129 已就緒；現在保持 HOME，等待 ROS 命令。`
機器人會停在 HOME 姿態，直到你送出第一個指令才會動。

查狀態／停止：

```bash
bash tools/status_robot129_ros_webrtc.sh
bash tools/stop_robot129_ros_webrtc.sh
```

`start_robot129_ros_webrtc.sh` 如果偵測到已經在跑，會直接沿用現有 session、**不會重開**——
所以如果模擬器已經在跑，手臂／夾爪／場景物件會停在上一次測試結束的位置，不是乾淨的 HOME。
每次要從乾淨狀態開始測試（手臂回 HOME、場景物件回到初始位置），用這個代替：

```bash
bash tools/restart_robot129_ros_webrtc.sh
```

它就是「先 stop 再 start」，會強制重開一份全新的 Isaac Sim process。

**注意（共用機器）**：這台 server 同時間只能跑一個 Isaac/WebRTC session（固定佔用 port 49100/47998），
如果別人也在用（例如障礙迴避 demo），你的 `start_robot129_ros_webrtc.sh` 會直接失敗並印出被誰占用。
跑之前先問一下有沒有人在用，或用 `bash tools/status_robot129_ros_webrtc.sh` 自己確認。

## 4. WebRTC 怎麼看畫面

用 NVIDIA Isaac Sim WebRTC Streaming Client 連線：

- Server IP：`140.113.203.85`
- Signaling port：`49100`
- Stream port：`47998`

啟動 `start_robot129_ros_webrtc.sh` 的同時就會啟動 WebRTC，畫面會持續串流；
你呼叫 `set_joint_positions` / `open_gripper` / `close_gripper` 時，畫面裡的手臂會即時動。

## 5. 完整範例（可直接跑）

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
bash tools/run_robot129_control_demo.sh
```

這個指令會先確認模擬器有在跑（沒有就啟動），再依序執行：

```python
set_joint_positions([0.0, 1.20, -1.25, 0.0, 0.15, 0.0])
close_gripper()
set_joint_positions([0.2, 1.35, -1.55, 0.3, -0.20, 0.10])
open_gripper()
```

結果會寫到 `out/ros_webrtc_robot129/control_api_demo.json`，同時可以在 WebRTC 畫面即時看到手臂動作。

## 6. 底層說明（不需要知道，但萬一要 debug）

`robot129_control_api.py` 就是把你的數值包成 ROS 2 `JointTrajectory` 訊息，
發布到既有的 topic，沒有另外做一套控制系統：

- `set_joint_positions` → `/robot129_sim/arm_controller/joint_trajectory`（joint1..joint6）
- `set_gripper` / `open_gripper` / `close_gripper` → `/robot129_sim/gripper_controller/joint_trajectory`（joint7；joint8 由 Isaac bridge 自動鏡射為 `-joint7`）
- 到位確認讀 `/robot129_sim/joint_states`

完整架構、topic 列表與開發用兩終端機流程見 [`docs/LOCAL_ROS_CONTROL.md`](LOCAL_ROS_CONTROL.md)。
