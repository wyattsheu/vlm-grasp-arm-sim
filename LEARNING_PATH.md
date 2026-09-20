# 11 課建立 Robot 129 模擬系統

每課只在上一課驗收完成後開始。時間是參考值；以理解與可重現性為準。

## Lesson 1 — 交接包與 server 基線

理解 workspace、第三方來源、CPU/GPU、driver、容器、ROS distro 與版本鎖定。驗證 checksum，產生 server environment manifest，決定 Isaac Sim 安裝方式和 ROS 配置。通過條件：環境資訊完整、磁碟預算明確、選定版本有官方相容證據；尚未安裝的項目標為 NOT RUN。

## Lesson 2 — 用官方機器人理解 Isaac Sim

用官方示例學 stage、prim、rigid body、articulation、joint drive、timeline、reset 和 headless script。先操作官方 manipulator，不使用 Piper。通過條件：能從乾淨啟動重現一個 articulation，讀取關節狀態並解釋 reset 後的狀態。

## Lesson 3 — 讀懂 Piper URDF 與 TF tree

在 RViz／URDF 工具中理解 link、joint、origin、axis、limit、visual、collision、inertial。畫出 base_link 到 link6、gripper_base、兩指的 tree，核對 `robot/arm_parameter_summary.yaml`。通過條件：無缺 mesh，所有關節名稱／方向／單位有一份人工核對表，未知項目沒有被猜值填補。

## Lesson 4 — 建立乾淨的 robot129_description

建立新的 ROS description package，從 vendor 資產引用或可追溯地複製。加入明確的 flange、tool0、tcp、camera mount 和 optical frames；處理 fixed joint preservation。通過條件：URDF/Xacro 可重現展開、tree 唯一、frame 命名與 transform direction 文件一致。

## Lesson 5 — 匯入 Piper USD 與運動學驗證

以版本化 importer 設定產生 USD，固定底座，檢查 articulation root、單位、關節、limits 和 drives。用多組關節姿態比較 ROS FK 與 Isaac link pose。通過條件：reset 穩定、各關節方向正確，FK 差異在預先設定的容許範圍內，匯入步驟可由腳本重建。

## Lesson 6 — 夾爪、碰撞與接觸

釐清 joint7／joint8 耦合、單指行程與總開口，建立 visual／collision 分層，設定指尖 contact material、物體 mass/inertia。先單獨開合，再做固定物體接觸測試。通過條件：兩指對稱、無爆炸或穿透、力／摩擦／gains 有來源或明確標示估計。

## Lesson 7 — wrist RGB-D、CameraInfo 與 SceneBundle

加入相機 prim 與 ROS optical frame，輸出 RGB、depth、CameraInfo、TF、JointState。用已知平面／方塊驗證投影、depth 定義、尺度與對齊，再寫出 SceneBundle。通過條件：現有 `validate_scene.py` 通過，timestamp／frame／depth metadata 可追溯。

## Lesson 8 — ROS 2 Bridge 與 ros2_control

理解 `/clock`、QoS、namespace、ROS domain、state 與 command。先啟用 isolated simulation domain，再建立 joint state broadcaster 和 trajectory／gripper controllers。通過條件：只控制模擬 articulation，已知軌跡追蹤可量測，controller reset／cancel／timeout 有測試。

## Lesson 9 — MoveIt 2 與任務分解

建立 robot129 MoveIt config、planning groups、TCP、self-collision matrix、controller mapping 和 PlanningScene。再以 MoveIt Task Constructor 表達 approach、grasp、lift、transit、place、release、retreat。通過條件：先完成無 VLM 的固定目標規劃，整條 arm／camera／payload 路徑接受碰撞檢查。

## Lesson 10 — 接入 MPG 與端到端模擬

先跑 replay，再選擇模型 backend。把 SceneBundle 餵給 grounding／lifting，將語意點轉成明確的 object reference 與 TCP pose，由任務層執行並重新觀測。通過條件：固定 seed 可重現；grounding、geometry、planning、controller、physics failure 分開記錄；物體完成 lift、hold、transport、release、settle 驗收。

## Lesson 11 — Thor 連模擬器與 sim-to-real rehearsal

在獨立測試網路／domain，於 Thor 重新建置預計部署的 ARM 程式，只連 server 的模擬 observation／command adapter。量測 VLM latency、memory、DDS、clock 與 stale data handling。通過條件：不啟動實機 driver，server-only 與 Thor-connected 結果可比較，整理正式實機驗證前的差異與剩餘風險。

