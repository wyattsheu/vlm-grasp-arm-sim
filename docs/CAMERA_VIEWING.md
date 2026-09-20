# Robot 129 腕上 RGB-D 相機觀看與驗證

更新：2026-09-15

## 兩個畫面不是同一台相機

| 畫面 | 用途 |
|---|---|
| WebRTC viewport | 從外面觀看整支機器人，可旋轉與縮放 |
| Wrist RGB-D viewer | 模擬夾爪末端相機看到的 RGB 與 depth，會跟著手臂移動 |

腕上相機附著在以下末端 frame：

    gripper_base
      └── camera_mount
          └── camera_link
              └── camera_color_optical_frame
                  └── Isaac RGB-D sensor

目前是模擬用 nominal mount。真實 Robot 129 的手眼外參尚未量測，因此 physical_wrist_extrinsic_verified=false。

## 階段一：啟動 Isaac 與 viewer

在 PRO 6000 server：

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    bash tools/start_robot129_ros_webrtc.sh
    bash tools/start_robot129_camera_viewer.sh

viewer 只綁定 127.0.0.1:8090，沒有公開到 server 網路。

## 階段二：從自己的電腦觀看

在自己的電腦另開一個 terminal：

    ssh -N -L 8090:127.0.0.1:8090 wyattsheu@140.113.203.85

保持 terminal 開著，瀏覽器進入：

    http://127.0.0.1:8090

左邊是 RGB，右邊是對齊的 depth。深度圖藍色較近、紅色較遠。頁面下方會顯示 frame、尺寸、中心深度、有效比例和 camera world position。

## 階段三：讓手臂移動並觀察視角

在 server 另一個 terminal：

    cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
    bash tools/send_robot129_ros_pose.sh inspect --duration 2
    bash tools/send_robot129_ros_pose.sh home --duration 3

移動時：

- WebRTC viewport 顯示整支手臂。
- localhost:8090 顯示末端相機視角。
- camera world position 與畫面內容應一起改變。

## 階段四：用 ROS 確認資料

    bash tools/robot129_ros.sh topic list -t
    bash tools/robot129_ros.sh topic info /robot129_sim/camera/color/image_raw -v
    bash tools/robot129_ros.sh topic echo /robot129_sim/camera/aligned_depth_to_color/camera_info --once
    timeout 8s bash tools/robot129_ros.sh topic hz /robot129_sim/camera/color/image_raw
    timeout 8s bash tools/robot129_ros.sh topic hz /robot129_sim/camera/aligned_depth_to_color/image_raw

介面：

| Topic | ROS type | 格式 |
|---|---|---|
| /robot129_sim/camera/color/image_raw | sensor_msgs/msg/Image | rgb8, 640x480 |
| /robot129_sim/camera/aligned_depth_to_color/image_raw | sensor_msgs/msg/Image | 32FC1 meter, 640x480 |
| /robot129_sim/camera/aligned_depth_to_color/camera_info | sensor_msgs/msg/CameraInfo | K, D, R, P |
| /tf | tf2_msgs/msg/TFMessage | world 到 camera_color_optical_frame |

目前在同時執行 RTX、WebRTC 與兩張 raw 影像時，實測約 4.3 到 4.6 Hz。這是目前效能量測，不是設定宣稱的 30 Hz。

## 階段五：離線檢查 RGB 與 depth 同一像素

    python3 tools/inspect_robot129_rgbd.py \
      --scene out/ros_webrtc_robot129/wrist_camera \
      --output out/camera_inspection/wrist_live

開啟：

    out/camera_inspection/wrist_live/rgbd_preview.png
    out/camera_inspection/wrist_live/rgbd_report.json
    out/camera_inspection/wrist_target_validation.json

目前驗證結果：

- RGB 與 depth 都是 640x480。
- frame_id 是 camera_color_optical_frame。
- 紅色方塊 bbox 是 [278,94,361,181]。
- 方塊 depth 中位數約 0.359 m。
- 背景 depth 中位數約 0.395 m。
- RGB 與 depth 使用同一 pixel grid。

## 停止

只停止相機網頁：

    bash tools/stop_robot129_camera_viewer.sh

停止 Isaac 並釋放 GPU：

    bash tools/stop_robot129_ros_webrtc.sh

viewer 不啟動 vLLM，也不連接 CAN、Piper SDK、RealSense USB 或 production ROS domain。
