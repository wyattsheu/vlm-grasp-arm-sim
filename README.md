# Robot 129 PRO 6000 simulation handoff

這個 handoff workspace 已在 PRO 6000 server 完成 simulation-only 建置。它包含 Robot 129/Piper 幾何、Isaac USD 與 PhysX 場景、隔離的 ROS 2 控制、MoveIt/MTC 驗收、RGB-D SceneBundle、MPG replay、WebRTC 與上層 vLLM 學習環境。

初次學習請從 [`docs/BEGINNER_TUTORIAL.md`](docs/BEGINNER_TUTORIAL.md) 開始；它使用目前已建好的系統，不會要求重新安裝 Isaac Sim。

若只想把外部算法的六軸／夾爪輸出接進來測試，不需要處理 ROS／Isaac 細節，見
[`docs/CONTROL_API_FOR_SENIOR.md`](docs/CONTROL_API_FOR_SENIOR.md)（`set_joint_positions` / `open_gripper` / `close_gripper`）。

目前可以直接做：

- 驗證交接包完整性。
- 盤點 server 環境，產生不含 secrets 的環境報告。
- 閱讀及視覺化 Piper URDF／mesh。
- 執行 MPG 離線單元測試與 replay acceptance test。
- 依 `LEARNING_PATH.md` 建立 Isaac Sim、ROS 2、MoveIt 與模擬場景。

仍未驗證的範圍是真實 Robot 129 hardware revision、相機內外參、實機 controller/contact 參數與 Thor 跨主機 rehearsal。CAN、Piper SDK、RealSense USB 與 production ROS domain 沒有連接。

## 搬到 server 後先做什麼

1. 將整個目錄放在 server 的獨立可寫 workspace，保留內部相對路徑。
2. 執行 `bash tools/verify_bundle.sh`。
3. 執行 `bash tools/collect_server_info.sh`；結果會寫到 `environment/server_environment.txt`。
4. 開啟 Codex 或 Claude Code，將 `START_PROMPT_FOR_CODEX_OR_CLAUDE.md` 全文貼給它。
5. 一次完成 `LEARNING_PATH.md` 的一個 lesson，每課留下可重現的輸出與學習筆記。

建議把這個目錄初始化成新的 Git repository；先完成 checksum 驗證，再建立初始 commit。不要把 `.env`、權重、cache、Isaac asset cache、ROS `build/install/log` 加入版本控制。

## 目錄地圖

```text
AGENTS.md / CLAUDE.md                 助理工作規則
START_PROMPT_FOR_CODEX_OR_CLAUDE.md  可直接貼上的啟動提示詞
LEARNING_PATH.md                     11 課逐步建立路線
docs/                                搬移計畫與既有介面文件
robot/vendor/piper_description/      Piper URDF、Xacro、STL（原樣副本）
robot/reference_moveit_config/       MoveIt 設定參考；尚未驗證於 Isaac
robot/arm_parameter_summary.yaml     URDF 參數摘要與缺口
calibration/                         相機、外參、夾爪 mapping 模板
environment/                         server／Thor 環境資訊與版本決策
research/                            MPG 核心、prompt、runner 與 tests
provenance/                          授權、來源及 SHA-256
tools/                               唯讀盤點與完整性驗證工具
```

`robot/reference_moveit_config` 是來源參考，不可直接假設 controller、tip link 或 gripper mapping 正確。`piper.ros2_control.xacro` 使用 mock hardware；Isaac 的控制設定需在教學流程中另外建立。

## 通用 USD WebRTC 檢視器（2026-09-16 新增）

不限於 Robot 129，任何 `.usd`/`.usda`/`.usdc` 檔案都可以用這個工具直接透過 WebRTC 看：載入指定檔案、自動排除場景本身的 ground plane/physics scene 再算 bounding box、自動對準攝影機、加上深灰底、燈光、1m 格線與世界座標軸（`isaacsim.util.debug_draw`）、放一個 10cm 參考方塊，並在終端機印出檢查報告（單位、實際尺寸、prim 樹、mesh 面數/退化面、材質是否遺失、physics API 是否存在）。可選 `--physics` 讓它掉到地板上跑幾秒看會不會爆炸或穿透。

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
bash tools/start_usd_webrtc.sh --usd /絕對路徑/my_model.usd [--physics]
# 連線資訊同 Robot 129 viewer：
#   Server IP:      140.113.203.85
#   Signaling port: 49100
#   Stream port:    47998
bash tools/status_usd_webrtc.sh   # 看目前狀態與最新的 report.json
bash tools/stop_usd_webrtc.sh
```

signaling/stream port 和 Robot 129 的 WebRTC viewer 共用同一組固定 port（49100/47998），設計上同時間只跑一個；`start_usd_webrtc.sh` 會先檢查 port 有沒有人在用，若有會直接失敗並提示，不會搶走正在使用中的 session——用完那個 session 記得手動停掉，這邊才啟動得了。

**連線後怎麼操作**（這幾個都是去查 Kit 原始碼確認過的實際按鍵，不是憑印象猜的）：
- **移動視角（WASD）**：要先按住**滑鼠右鍵**不放，這時候才會進入飛行模式，WASD 才會動。單獨按 WASD 沒有任何反應是正常的——這是 Omniverse／Unreal 那種「按住右鍵才能飛」的操作慣例，不是壞掉。
- **用滑鼠拉／推物體**：要按住 **Shift** 再用**滑鼠左鍵拖曳**在物體上（不是單純拖曳，一定要按著 Shift）。這個互動功能在 `omni.physx.ui` 這個擴充套件裡,已經加進 `start_usd_webrtc.sh` 的啟動參數讓它一定會載入。畫面連上時物理是持續在跑的（不是凍結的截圖），可以直接試。

實作、驗證與已知限制的細節記在 memory `usd-webrtc-viewer` 這個檔案裡（包含好幾個開發過程中踩到的坑：Python 陷阱、bbox 排除 ground plane 與 PointInstancer 的作法、以及上面這兩個操作方式怎麼從原始碼查到的）。程式本身在 `sim/scripts/view_usd_webrtc.py`，已經實測跑過 WebRTC 全流程（不是只測過離線模式）。

## 離線研究程式快速驗證

在 Python 環境已有 NumPy、Pillow、PyYAML、Requests 後：

```bash
cd research
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 -m unittest discover -s tests -v
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 scripts/validate_offline_pipeline.py
```

第二個命令使用合成 fixture／replay，不會呼叫 VLM、ROS 或機器人。需要的詳細說明在 `docs/offline_pipeline.md`。

## 交接包版本

- 建立日期：2026-09-13
- 來源機器：Robot 129 / Jetson AGX Thor
- 用途：PRO 6000 simulation learning and development bootstrap
- 狀態：資產與離線程式已打包；Isaac／ROS 模擬建置 NOT RUN

## 2026-09-20：這個 repo 實際 push 到 GitHub 的範圍

Push 上去的只有「重要」的原始碼與文件，不是整個 workspace（357M 中大部分是可重新產生的建置產物或不該公開的東西）。已排除：

| 排除 | 理由 |
|---|---|
| `ros2_ws/build`／`install`／`log` | colcon 建置產物，`colcon build` 重新產生 |
| `ros2_ws/src/external/`（moveit_task_constructor、py_binding_tools） | 第三方 ROS 套件，各自有自己的上游 repo，不是本專案程式碼 |
| `robot/vendor/piper_description` | 原廠 Piper 描述套件的參考副本；實際會用到的 mesh 已經包含在 `ros2_ws/src/robot129_description/`（見下方授權說明） |
| `sim/assets/robot129/robot129.usda`（26MB） | 由 `tools/import_robot129_usd.sh` 從 URDF 轉出的產物，可重新產生，不是原始碼 |
| `out/`、`research/out`、`research/cache`、`research/logs` | 每次執行的錄影、報告、VLM 回應快取，資料不是程式碼 |
| `deployment/` | 內含 `thor_access_inventory.yaml`，記錄了真實 Thor 主機的 SSH hostname——即使 repo 是 private 也不放真實主機連線資訊上雲端 |
| `references/upstream/` | 其他專案的參考副本，授權狀態未逐一確認，先不放 |
| `lessons/`、`tutorials/` | 這個 workspace 的學習教材，跟「機器手臂模擬與開發」核心程式碼關係較遠，先不放 |

**手臂 URDF mesh 授權**：`ros2_ws/src/robot129_description/meshes/*.STL`（13 個檔案，約 10MB）已確認來自 `provenance/PIPER_LICENSE` 記載的上游 Piper description 套件，**MIT License**（Copyright RosenYin），可以公開重新散布，已隨 repo 一起 push。

