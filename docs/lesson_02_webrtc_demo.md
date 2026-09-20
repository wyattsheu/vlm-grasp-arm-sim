# Lesson 2 WebRTC Cartpole Demo

## 目的

這個 Demo 將 Lesson 2 的官方 Cartpole articulation 保持運行，並透過既有 Isaac Sim 6.0.1 WebRTC extension 傳送 Kit viewport。它不安裝套件、不啟動 ROS，也不連接任何機器人硬體。

## 啟動

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    bash tools/start_lesson_02_webrtc.sh

啟動器會：

1. 確認本帳號沒有另一個 Isaac process。
2. 依 GPU utilization 與已用 VRAM 選擇較低負載的一張卡，不等待空閒門檻。
3. 以 `--livestream 2` 啟動 private-network WebRTC。
4. 等待 Isaac 回報 READY，並確認 TCP signaling port 49100 已監聽。
5. 將日誌寫到 `out/lesson_02/webrtc/server.log`。

## WebRTC Client

在已經使用過的 NVIDIA Isaac Sim WebRTC Streaming Client 中輸入：

    Server: 140.113.203.85
    Signaling port: 49100

若你的 Client 是從另一張網卡／VPN 進入，伺服器也有 `140.113.203.84`、`100.96.51.38`、`192.168.2.146`；使用從 Client 所在網路能到達的位址。stream port 為 47998，通常由 Client 協商，不需要另外輸入。

連線後應看到白色網格地面、淡黃色水平軌道、淡藍色 cart 與紫灰色 pole。程式持續對 `slider_to_cart` 施加正弦 effort；小車和長桿會運動，每 600 steps reset 一次。

## 狀態與日誌

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    bash tools/status_lesson_02_webrtc.sh
    tail -f out/lesson_02/webrtc/server.log

日誌中的 `READY`、`Stage: /World/Robot` 和兩個 joint 名稱證明正確場景已載入。按 `Ctrl+C` 可退出 `tail -f`，不會停止 Isaac。

## 停止

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    bash tools/stop_lesson_02_webrtc.sh

停止器只向 PID file 所記錄的這次 Demo process 傳送 TERM，不使用 `pkill`，也不影響其他使用者的 GPU 工作。

## 本次驗收

- `omni.kit.livestream.core-10.2.1`：startup
- `omni.kit.livestream.webrtc-10.3.2`：startup
- `omni.kit.livestream.app-10.1.1`：startup
- Isaac：`app ready`
- Lesson scene：`[WEBRTC DEMO] READY`
- TCP signaling：`0.0.0.0:49100` listening
- Cartpole reset：已觀察到多次 600-step reset

本次沒有從外部 Client 建立 session，因此端到端影像連線仍需在 Client 按 Connect 時確認。
