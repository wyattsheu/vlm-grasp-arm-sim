# Robot 129 → PRO 6000 模擬開發環境搬移計畫

日期：2026-09-13  
狀態：DESIGN ONLY。依據本次對話及先前唯讀盤點整理；本輪沒有重新深入讀取原始碼、擷取硬體參數、複製模型或安裝軟體。文件中的新目錄、檔名與設定皆為提案，除明確指出的既有來源外，不表示檔案已建立。

## 1. 目標與範圍

在 PRO 6000 server 建立完整開發環境：Isaac Sim、ROS 2、運動規劃、VLM／MPG、場景與評估。先完成固定底座 Piper + 平行夾爪 + wrist RGB-D 的取放測試，再以 Thor 執行預計部署的軟體、連接 server 模擬器，最後由人員進行實機驗證。

第一版不包含 Kachaka 移動導航、全身協同控制、強化學習訓練或完整實驗室重建。底座與相機支架即使不動，仍需有足以檢查碰撞的幾何模型。

本計畫不等於實作授權。既有 AGENTS.md 的階段門檻仍適用；未來模擬用控制程式需要明確的 simulation-only 授權與網路邊界，不能把原本 Phase 5 的「只算 IK」當成模擬執行許可。物理執行仍由 Wyatt／Tuan 負責。

## 2. 建議部署架構

| 層 | PRO 6000 開發期 | 部署前驗證 | 最後實機階段 |
|---|---|---|---|
| 世界／感測器 | Isaac Sim 場景、虛擬 RGB-D、接觸 | 同左 | 真實桌面、RealSense |
| 控制後端 | 模擬 articulation／controller | server 模擬後端 | Thor 上 Piper driver |
| 規劃／任務 | server 的 ROS 2、MoveIt、任務流程 | 指定部署節點在 Thor 執行 | Thor 或明確定義的外部服務 |
| VLM／MPG | 先在 server 執行 | 在 Thor 測實際資源與延遲 | 依驗證結果部署 |
| 評估 | server 保存每次實驗證據 | 同左，另量測通訊與端到端延遲 | 真實 held-out 場景獨立評估 |

應建立三個獨立依賴環境：Isaac Sim、ROS／研究程式、VLM inference。透過標準 ROS 訊息、既有 SceneBundle 與版本化 plan 契約連接。不要讓研究核心模組依賴 Isaac Python API。

前期資料流：

```text
Isaac RGB-D + CameraInfo + TF + JointState
             ↓
        SceneBundle adapter
             ↓
     共用 MPG grounding / lifting
             ↓
  幾何提案 → TCP pose / grasp adapter
             ↓
       任務流程 + MoveIt
             ↓
  trajectory controller → 模擬手臂
             ↓
  重新觀測 + 物體狀態驗收 + 結果紀錄
```

三個語意點不等於三個運動指令。必須明確展開 approach、close、lift、transit、preplace、release、retreat，並处理每一步的失敗、取消與逾時。

## 3. 搬移前需要準備的資料

### 3.1 最小可開工資料包

| 項目 | 用途 | 目前已知情況 | 後續動作 |
|---|---|---|---|
| Piper 精確型號、硬體版本、夾爪版本 | 選對模型與規格 | 僅確認為 Piper 六軸＋平行夾爪 | Wyatt／Tuan 提供銘牌或規格 |
| URDF／Xacro + 所有 mesh | 建立運動鏈與模型 | 先前已找到 piper_description | 實作階段核對版本、完整打包 |
| MoveIt SRDF、controller、joint limits | 規劃設定起點 | 先前已找到 with-gripper 配置 | 作參考副本，重新驗收 |
| 相機內參與串流設定 | 重建視角與深度契約 | 實際使用值仍待快照 | 由經核准的擷取取得 |
| flange／TCP／camera 外參 | 正確定位手爪與相機 | 先前找到 hard-coded ee_T_cam | 確認來源 frame 與校正精度 |
| 手臂、相機、支架與桌面尺寸 | 補齊真實場景 | 尚未整理 | 提供量測表與多角度照片 |
| 共用 MPG 原始碼、設定、測試 fixtures | 搬移研究流程 | 目前 workspace 已有 | 匯出明確版本與必要檔案 |
| 少量真實 RGB-D SceneBundle | 檢查模擬與實機差異 | 完整可用性待驗證 | 先準備至少一組可驗證資料 |
| server 環境資訊 | 選定安裝方法 | 使用者表示有 PRO 6000 | 確認完整 GPU 名、OS、driver、資源 |

若真實相機資料尚未取得，可先做明確標記的示例相機，但不能稱作已校正的 Robot 129 數位分身。

### 3.2 主機／軟體環境紀錄

後續在兩台主機各建立 environment_manifest，記錄：CPU 架構、OS、GPU 完整名稱與 VRAM、driver、RAM、可用磁碟、Docker／Container Toolkit、ROS distro、RMW、Python、模型服務版本、必要套件版本。

PRO 6000 的完整產品名稱及 VRAM 尚未核對，不能單憑名稱推定為哪一代卡。伺服器是否有本地螢幕、是否只能 SSH、是否可用串流 viewer，也需先確認。

前次討論的候選組合是 Isaac Sim 6.1.0、ROS 2 Humble；實際安裝前核對正式 release、相容性與既有 driver，再固定版本。Ubuntu 22.04／Humble 與 Ubuntu 24.04／Jazzy 應分別評估；若 server 已有系統，優先使用匹配的容器配置，避免為本計畫重裝共享主機。初期不依賴跨 ROS distro 的通訊。

環境匯出不得包含 API key、完整環境變數、認證目錄或私有服務憑證。只記錄需要哪些 secret 名稱，由目的主機自行注入。

### 3.3 手臂參數表

| 參數群 | 必填内容 | 驗證方法 |
|---|---|---|
| 身分 | 型號、revision、夾爪型號、資產來源版本 | 對照銘牌與原廠 |
| 運動學 | joint 名称／順序／型態／axis／origin、零位、正方向 | URDF 與已知姿態 FK 比對 |
| 限制 | 位置、速度、加速度、effort 限制與單位 | 原廠資料＋部署設定；不把通用值當實機額定值 |
| 動力學 | 每 link mass、COM、inertia、payload | 原廠／CAD；缺值需標示估計 |
| 末端 | flange frame、TCP frame、固定 transform、工具重量 | 實測／設計圖 |
| 夾爪 | 最大開口、單指行程、兩指耦合、速度、力限制 | 幾何與回授對照 |
| 控制 | position／velocity／effort 模式、頻率、插補、延遲 | 先盤點設定，後續隔離測試 |
| 校正 | joint offset、校正版本、已知誤差 | 帶日期的量測紀錄 |

必須分清楚「手爪總開口寬度」「單指 prismatic 位移」「driver gripper 欄位」。不能在尚未驗證前一律除以二或直接視為公尺。

### 3.4 相機與安裝參數表

- 每台相機的角色：wrist／固定觀察／僅供錄影；初期先完成 wrist。
- RGB/depth resolution、FPS、encoding、depth_scale_m、近遠裁切範圍。
- 內參 K、畸變 D、rectification R、projection P；參數必須對應實際輸出解析度。
- depth 是否已對齊 RGB、對齊後對應哪個 optical frame。
- flange→mount→camera_link→optical frame 的 transform 與方向。
- 相機及支架外形尺寸、重量、安裝方向與碰撞外形。
- 校正來源、日期、誤差；分清楚 nominal CAD 外參與 calibrated 外參。
- timestamp 的時間來源、RGB/depth skew、joint age、TF 取樣時間。

不要把 viewer 顯示尺寸或 launch 預設值當成已驗證的 raw camera 設定。不要把「相機深度修正」當作「相機安裝位移」。

## 4. 既有資產位置與使用方式

以下路徑來自先前盤點，本輪未重讀；實作前要確認當前檔案與 revision。

```text
/home/wyattsheu/workspaces/robotic_agent/mm_system/piper_ros/src/
  piper_description/urdf/piper_description.urdf
  piper_description/urdf/piper_description.xacro
  piper_description/meshes/
  piper_moveit/piper_with_gripper_moveit/config/

/home/wyattsheu/workspaces/robotic_agent/mm_system/main_ws/src/mm_actions/mm_actions/
  actions/base_action.py         # 先前找到 ee_T_cam 的位置
  motion/piper_kinematic.py      # FK/IK 比對來源，不執行控制

/home/wyattsheu/workspaces/next_arm_tesk/
  src/mpg/
  scripts/
  configs/
  prompts/
  tests/
  docs/interface_contract.md
```

先前的重要證據：URDF 六軸定義從 piper_description.urdf:69 起，夾爪 joint7／joint8 在 :425／:483；with-gripper 的 piper.ros2_control.xacro:9 使用 mock_components/GenericSystem；ros2_controllers.yaml:19 列 arm joints，:34 只列 joint7；piper.srdf:13 的 arm tip 是 link6。這些都是待驗收的來源設定，不是已完成的 Isaac 模型。

現有 mm_actions_node.py:188 將 JointState 發到 /joint_states 作命令，:174 訂閱 joint_states_feedback；移植時要隔離這個 legacy 契約與新標準狀態 topic。base_action.py:167 的外參與 :177 的 FK frame 關係須一併確認。

任何來源副本都只寫入核准的 workspace，附來源路徑、commit／檔案 hash 與 license。來源可能有未提交修改，因此 commit hash 不能替代逐檔 hash。禁止改動 live-mounted source。

## 5. 手臂 3D 模型怎麼取得與處理

### 5.1 優先順序

1. 先使用已找到的 Piper URDF＋STL，對照實機型號與零位。這是最省時間的起點。
2. 若版本不符，再向原廠取得相同 revision 的 ROS description 或 CAD／STEP；要求包含夾爪與尺寸單位。
3. 缺少相機支架時，以尺寸量測建立簡化 CAD／collision geometry；視覺外觀後補。
4. 不易取得的桌面物品先使用已知尺寸的簡單幾何，再換成有授權、可追溯的實物模型。

### 5.2 三種檔案的角色

| 格式 | 包含什麼 | 還缺什麼 |
|---|---|---|
| STL／OBJ | 外觀幾何；格式支援的材質資訊有所不同 | 通常沒有完整 joint tree、關節限制、質量與控制 |
| STEP／CAD assembly | 精確零件與裝配幾何，可能有材料／質量資料 | 需轉 mesh 並建立可用的機器人關節描述 |
| URDF＋mesh | links、joints、幾何、可用的慣量與限制 | Isaac 的 drives、接觸材質、sensor／場景設定需驗收 |
| USD | Isaac 使用的場景、模型、物理與感測器配置 | 正確性取決於匯入與驗證，不能只看外觀 |

如果只有整支手臂一個 STL，不能期待 importer 自動辨識六個關節。必須先拆成各剛體 link、定義旋轉軸與原點、建立 tree，才有可動的機器人。若有完整 URDF，就通常不需要重新畫整支手臂。

### 5.3 預定建置流程

```text
確認型號與資產授權
 → 整理 URDF/Xacro + meshes
 → 補 TCP、相機與支架 frame
 → 驗證尺寸、joint axes、limits、mass/inertia
 → 匯入 USD，固定底座
 → 設定 drives、finger coupling、contact materials
 → 單關節測試與 FK 對照
 → 無 VLM 的固定物體取放
 → 接 RGB-D 與研究流程
```

維護原則：URDF/Xacro 作為機器人運動鏈的主要來源；匯入產生的 USD 不手改。將 Isaac 專用物理、材質與感測器設定放在額外 USD layer／設定檔。需要固定相機／TCP link 的保留與合併策略，避免 importer 合併 fixed joints 後失去 frame 對應。

視覺 mesh 與 collision mesh 分開。手臂使用簡化碰撞外形，指尖保留足夠精度；碗或容器必須保留開口，不能用單一 convex hull 把內部填滿。碰撞模型也需包含相機、支架與攜帶物體。

初期先固定一種物理 backend 與 timestep。慣量、friction、gains 若未量測，標記為估計，不透過無限制增加力或摩擦掩蓋模型錯誤。

## 6. 資料與介面契約

### 6.1 共用觀測資料

沿用 SceneBundle：rgb.png、depth.png 或 depth.npy、camera_info.json、tf.json、joint_states.json、instruction.txt、manifest.json。既有 scene_bundle.py:64 支援兩種深度檔案；capture_scene.py:70 要求明確 depth scale。

模擬 metadata 額外記錄來源、simulator version、physics backend、scene／asset hash、seed、episode ID、simulation timestamp、camera model、noise config。是否擴充現有 schema 或放獨立 metadata 檔，於實作時確認相容性。

深度要驗證是 optical-axis Z 還是 camera-to-point 距離；目前 pinhole lifting 需要 optical-axis Z。RGB 與深度需對應相同像素網格，不能只因尺寸相同就稱為 aligned。初期可直接保存 float meters，legacy baseline 如需 uint16 則顯式轉換並紀錄量化。

### 6.2 座標、時間與狀態

- ROS 與 Isaac API 的座標／quaternion 順序皆在 adapter 顯式轉換；禁止直接搬四個數字。
- TF 的 transform direction 必須標記，例如 T_base_camera。
- 相機 optical frame、flange、TCP、物體參考點各自獨立。
- 以已知 3D 點投影／反投影測試驗證相機與 transform。
- 所有模擬 ROS 節點使用同一 /clock；wall-clock 用於推論延遲與操作 timeout，不能混入幾何 timestamp。
- reset 後處理時鐘回跳、舊 TF、controller trajectory 與舊 plan；每個 episode 用新的 ID。
- 每條 TF 邊只有一個權威發布者，避免 Isaac 與 robot_state_publisher 重複發布。
- QoS、topic name、frequency、encoding 均形成可驗證的 mapping 表。

### 6.3 研究與執行之間

現有 Plan3D 契約仍是草案，不能把 surface point 當 TCP pose。executor adapter 必須補 grasp orientation、object-to-TCP offset、payload collision representation，再交給規劃器。

評估至少區分三種狀態：語意 grounding 是否正確、幾何／路徑是否可行、物體是否實際完成任務。FollowJointTrajectory 完成不等於抓到物體。

legacy 單點流程與新 MPG 流程保留獨立入口。比較實驗固定場景、模型、controller 與 evaluator，避免一次改變所有層。

## 7. 預定檔案包與目錄

以下是未來要建立的交接結構，這次尚未打包：

```text
robot129_sim_handoff/
  README.md
  provenance/source_manifest.json
  provenance/SHA256SUMS
  provenance/licenses/
  environment/server_manifest.json
  environment/thor_manifest.json
  environment/version_matrix.yaml
  robot/description/             # URDF/Xacro、meshes、必要 package metadata
  robot/reference_configs/      # 原有 MoveIt 參考副本
  calibration/arm_parameters.yaml
  calibration/gripper_mapping.yaml
  calibration/camera_intrinsics.yaml
  calibration/extrinsics.yaml
  calibration/measurement_notes.md
  reference_scenes/              # 少量已驗證 RGB-D，附來源
  research/                     # MPG、prompts、configs、必要 tests
  docs/interfaces.md
  docs/acceptance_checklist.md
```

server 專案的提議結構：

```text
robot129_sim/
  src/mpg/                      # 共用研究核心
  sim/assets/                   # USD layers 與可追溯模型
  sim/scenes/                   # 桌面／物件配置
  sim/scripts/                  # 匯入、啟動、輸出資料；後續才實作
  ros2_ws/src/robot129_description/
  ros2_ws/src/robot129_moveit_config/
  ros2_ws/src/robot129_sim_bringup/
  ros2_ws/src/robot129_sim_adapter/
  ros2_ws/src/robot129_tasks/
  configs/                      # 執行 profiles、控制／感測器設定
  deployment/                   # 可重建的環境規格
  tests/                        # 契約、幾何、場景驗收
  docs/
  data/                         # 觀測、ground truth 分開存放
  out/                          # 每次實驗結果，不混入 source
```

不搬：.venv、ROS build/install/log、Docker 儲存目錄、全量模型 cache、完整共享 repo、secret、無關 teleop／X-VLA 資料。權重可記錄模型 ID／revision，按實際需要另外取得。

打包採 allowlist，保留相對路徑、必要的 mesh package 映射與 license；不能讓 symlink 指向來源機器外部路徑。目的主機收到後先驗 SHA-256，再檢查 mesh references 是否全數可解析。

## 8. 實施順序與驗收

| 階段 | 工作 | 通過條件 | 交付物 |
|---|---|---|---|
| M0 環境確認 | 取得 server 資訊，固定版本／路徑／網路 | 官方範例與相容性檢查可跑 | environment manifest |
| M1 資產整理 | 核對 Piper 版本、mesh、夾爪與 frame | 無缺 mesh，參數來源明確 | handoff package + hashes |
| M2 機構模擬 | 匯入 USD、drives、coupling | reset 穩定，joint directions／limits／FK 正確 | robot asset + kinematics report |
| M3 感測器 | wrist RGB-D、TF、SceneBundle | 已知幾何投影驗證通過、時間同步可量測 | validated simulated SceneBundle |
| M4 控制基線 | 模擬 controller、MoveIt、固定目標 | 軌跡、碰撞、取消／逾時測試通過 | controller / planner configs |
| M5 研究串接 | MPG + executor adapter | 固定 seed 可重現，失敗有分層原因 | run artifacts + results |
| M6 物理抓取 | 質量／接觸／payload 與閉迴路驗收 | 物體 lift、hold、place 達門檻 | physics grasp report |
| M7 Thor 連模擬器 | 在 ARM 重建部署軟體 | inference 延遲、memory、DDS 與時鐘驗證 | deployment rehearsal report |
| M8 人員實機驗證 | 真實 driver／相機與人員操作 | 按獨立實機計畫驗證 | real validation report |

M3 可以先串接離線 grounding，無須等待 M4／M6。M4／M6 先用明確的固定目標排除 VLM 因素；此時不得把固定目標實驗當成 VLM 成功率。

每一階段先定測試門檻再調參。任務尺度與實機需求尚未確認，因此此處不宣稱任意毫米／角度／秒數為正式標準。

## 9. 評估應記錄什麼

- 機構：FK 差異、trajectory tracking error、joint limits、超調、payload 下的穩定度。
- 感測器：intrinsics 投影誤差、depth 定義／尺度、frame 一致性、timestamp skew。
- Grounding：grasp／release 是否命中、abstention、parse failure、calls、latency。
- 規劃：IK、整條路徑的 arm／camera／payload 碰撞、未知空間處理。
- 任務：物體離桌、保持期間滑移、是否運送到位、釋放後是否在目的區域穩定。
- 系統：失聯、stale observation、時鐘 reset、推論逾時、取消與重試。

每次記錄 seed、資產與程式版本、原始觀測、模型回覆、計畫、trajectory、物體狀態及失敗階段。VLM 不應同步阻塞 physics callback；測試時明確採暫停等推論或持續模擬，記錄該政策。

ground truth 只提供給 evaluator；如果直接提供物體真實 pose／完整場景給 planner，標為 oracle baseline。模擬自動產生 labels 不能悄悄變成實際系統額外輸入。

assisted grasp 與純接觸抓取分開報告；PlanningScene 的 attached object 是規劃表示，不能誤認成 Isaac 中把物體固定到手上的物理 attachment。

模擬 noise 分兩步：先用理想深度驗證幾何，再加參數化噪聲／缺洞／延遲。沒有真實統計資料時只稱敏感度測試，不能稱為真實 D435 誤差模型。最後以真實 held-out 場景檢查 sim-to-real 差異。

## 10. 從 server 搬回 Thor

移轉同一份原始碼與設定版本，在 aarch64 重建依賴。x86 的 venv、ROS install 與容器 binary 不能直接複製使用；容器需有匹配的 ARM image／build，以及 Thor 平台所需的 CUDA／driver 相容條件。

切換邊界限定為 observation source、clock policy、controller backend、資源設定。Isaac runtime、USD 與場景產生工具留在 server。硬體 backend 必須獨立且顯式選擇，不因 topic 名相同就自動接實機。

在 Thor 連模擬器的 M7 使用專用測試 domain／network，僅啟動待測軟體副本，不讓既有硬體 driver 參與。ROS_DOMAIN_ID 與 namespace 是第一層隔離，不能取代網路隔離與硬體存取限制。server 不掛载 CAN／USB 裝置；需要跨主機時只開放測試端點。既有 production services 不由本計畫重啟。

## 11. 待補資訊與責任

| 誰 | 需要提供／確認 | 最晚需要時點 |
|---|---|---|
| Wyatt／server 管理者 | GPU 完整型號、OS、driver、資源、登入方式、可寫目錄、顯示／串流方式 | M0 |
| Wyatt／Tuan | Piper／夾爪精確版本、TCP 定義、允許使用的模型與規格 | M1 |
| Wyatt／Tuan | 相機、支架、桌面尺寸與安裝照片 | M2–M3 |
| 經核准的資料擷取 | 實際 CameraInfo、depth scale、TF／joint 快照與真實樣本 | M3 |
| 共同決定 | controller 契約、夾爪單位、grasp orientation 與驗收門檻 | M4 |
| Wyatt | 哪些節點最終要跑 Thor、模型版本與資源預算 | M7 前 |

現在不需要先購買或重建整支手臂模型。先核對現有 URDF＋STL 和實機版本，缺的通常是完整安裝幾何、TCP／相機 frame、夾爪映射與物理驗收。

## 12. 官方參考入口

以下是前次對話已查閱的官方入口；本輪不再查版本或下載資產。實作時改用選定版本的文件並核對 release。

- Isaac Sim requirements：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- ROS 2 reference architecture：https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/overview/ros2_reference_architecture.html
- ROS 2 installation：https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html
- URDF to pick-and-place：https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/tutorial_manipulator_workflow.html
- URDF importer：https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html
- ROS 2 control：https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_control.html
- Camera sensors：https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_camera.html
- MoveIt Task Constructor：https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html

## 本輪交付狀態

已完成：本搬移計畫與資料／參數／資產清單。  
NOT RUN：server 連線、參數快照、資產打包、模型下載、USD 匯入、安裝、ROS／VLM／IK／模擬執行。  
UNVERIFIED：server 環境、精確手臂版本、真實校正與物理模型精度。  
下一個具體工作包：M0＋M1 的唯讀環境與資產盤點，再依本清單建立可校驗的交接包。
