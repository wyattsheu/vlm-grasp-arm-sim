# Lesson 1 — 交接包與 PRO 6000 server 基線

日期：2026-09-13  
狀態：PASS（環境凍結、bundle 驗證與離線測試已完成）

## 這一課在建立什麼

這一課先建立可重現的「地基」。我們不啟動機器人、不匯入 USD，也不安裝另一套 Isaac Sim，而是回答四個問題：

1. handoff bundle 是否完整，來源檔案有沒有在搬移時損壞；
2. 目前 server 的硬體、OS、driver 和可用工具是什麼；
3. 哪一套 Isaac Sim／Isaac Lab 已經成功運作，真正的路徑與版本是什麼；
4. 與模擬器隔離的研究程式能否在純離線模式重現。

版本凍結的意思不是永遠不更新，而是先把「現在確定能工作的組合」記下來。之後若結果改變，我們才能判斷是程式、模型、資產還是環境版本造成。

## 架構位置

```text
PRO 6000 server
├── /mnt/HDD4/wyattsheu/IsaacLab
│   └── .venv/                 Isaac Lab 3.0.0 + Isaac Sim 6.0.1.0
├── /mnt/HDD4/wyattsheu/env_robot129_research
│   └── 離線 MPG／SceneBundle 研究環境，不載入 Isaac Sim
└── handoff/robot129_pro6000_sim_20260913
    ├── robot/vendor/          Piper 來源模型，唯讀
    ├── research/              離線研究程式與測試
    ├── environment/           本課建立的 server 與版本基線
    └── docs/progress/         每課的驗收報告

未來才建立：
├── ROS 2 Jazzy user-space／workspace 環境
└── VLM／vLLM 獨立環境
```

研究環境刻意放在 handoff bundle 外面。Python virtual environment 通常包含 symlink；若放進 bundle，`tools/verify_bundle.sh` 會正確地把它判定為破壞可搬移性。

## 今晚已代為完成的背景工作

### Bundle 完整性

執行：

```bash
bash tools/verify_bundle.sh
```

結果：所有列管檔案 SHA-256 相符；必要檔案存在；沒有 symlink、`.env`、`.pem` 或 `.key`。最終輸出為：

```text
PASS: required files, no symlinks, no obvious secret files, all checksums match
```

### Server 基線

執行：

```bash
bash tools/collect_server_info.sh
```

結果寫入 `environment/server_environment.txt`。已確認：

- CPU：AMD Ryzen Threadripper PRO 7955WX，16 cores／32 logical CPUs。
- OS：Ubuntu 24.04.3 LTS，kernel 6.14.0-37-generic，x86_64。
- GLIBC：2.39。
- GPU：2 張 NVIDIA RTX PRO 6000 Blackwell Max-Q Workstation Edition。
- VRAM：每張 97,887 MiB，約 96 GB。
- Driver：580.126.09。
- RAM：251 GiB；`/mnt/HDD4` 可用空間約 12 TiB。
- Docker CLI 28.5.1 存在，但目前帳號無 daemon 權限。
- Apptainer／Singularity 1.4.5、NVIDIA Container Toolkit 1.18.0、uv 0.12.11 可用。
- 目前 shell 找不到 `ros2`；沒有把 Docker 權限當成 blocker。

### 既有 Isaac 環境

沒有重新安裝或升級。從 source、lockfile、Python package metadata 與既有成功紀錄確認：

- Isaac Lab source release：3.0.0。
- Source path：`/mnt/HDD4/wyattsheu/IsaacLab`。
- Git branch：`feat/task1-rigid-asset-pipeline`。
- Git commit：`772533db073a1024fae8fa593c24288e24585efb`。
- Isaac Sim：6.0.1.0，以 pip packages 安裝在 IsaacLab 的 uv virtual environment。
- Isaac Python：3.12.14。
- Python executable：`/mnt/HDD4/wyattsheu/IsaacLab/.venv/bin/python`。
- Isaac Sim entry point：`/mnt/HDD4/wyattsheu/IsaacLab/.venv/bin/isaacsim`。
- 既有啟動 wrapper：`/mnt/HDD4/wyattsheu/IsaacLab/view.sh`。

handoff 原本的 `proposed: 6.1.0` 是早期 placeholder，已改成以本機成功環境 6.0.1.0 為準。

本課沒有重新啟動 Isaac Sim。盤點時兩張 GPU 的 utilization 都約 79%，分別已使用約 35.7 GB 與 31.1 GB VRAM，符合「GPU 忙碌時不要啟動」的停止條件。

注意：舊 `view.sh` 會先 `pkill` 同一套 Isaac virtual environment 的 Python process。Lesson 2 開始前要先建立不會自動終止其他工作的安全入口；現在不要自行執行它。

### 官方相容性決策

- 保留已成功運作的 Isaac Lab 3.0.0 + Isaac Sim 6.0.1.0 + Python 3.12。
- Physics backend 選 PhysX；Robot 129 資產尚未進入 physics 驗證。
- ROS 2 選 Jazzy 作為未來配置，因為 Ubuntu 24.04 + Jazzy 是 Isaac Sim 6.0 官方建議組合。ROS 仍為 NOT RUN。
- 單機 bridge 的初始 middleware 選 Fast DDS；Lesson 8 才實際驗證。
- MoveIt 選 ROS 2 Jazzy release line；精確套件版本要等建立 ROS 環境後記錄。
- 暫定 simulation domain `129`、namespace `/robot129_sim`，目前是 NOT VERIFIED；隔離證據完成前不得啟用模擬運動。

較晚更新的 Isaac Sim 6.0.1 requirements 頁列出 Linux 測試 driver 595.58.03；Isaac Lab 3.0 安裝頁列出 580.95.05 以上。現有 580.126.09 已成功執行本機環境，也符合後者，因此本課凍結現況，不更動共享 server driver。這項文件差異已寫入版本矩陣。

### 獨立研究環境與離線測試

建立：

```text
/mnt/HDD4/wyattsheu/env_robot129_research
```

Python 3.12.3，已安裝 NumPy 2.5.3、Pillow 12.3.0、PyYAML 6.0.3、Requests 2.34.2。環境約 85 MiB，不含模型權重。

單元測試結果：

```text
Ran 108 tests in 1.834s
OK
```

離線 acceptance 結果：

```text
status: PASS
scope: SYNTHETIC_REPLAY_ONLY
actual_model_calls: 0
robot_commands: 0
```

產物位於：

```text
research/out/validation/20260913T150415753964Z
```

`ab/overlay.png` 中，紅色方塊是合成的 grasp 目標，綠色方塊是 release 目的地，藍色點是 waypoint。這張圖證明離線資料流能讀取影像、解析 replay、畫出點位和保存結果；它不代表 VLM 準確，也不代表機器人已執行。

## Piper 3D 模型目前的身分

`robot/vendor/piper_description/` 已包含 Piper URDF、Xacro 與各 link 的 STL mesh。來源紀錄指向 `agilexrobotics/piper_ros` 工作副本，並保留 MIT license 與逐檔 SHA-256；AgileX Robotics 的官方 GitHub 也提供 Piper ROS／URDF 資產。

目前仍不能宣稱 bundle 中的檔案與 Robot 129 現場手臂完全相符，原因是：

- bundle 沒有記錄實機銘牌型號、Piper revision 與夾爪 revision；
- 來源工作樹在打包前是 dirty；
- 套件 `package.xml` 的描述與 license 欄位仍是 TODO；
- 尚未將 bundle hash 與目前官方 repository 的同 revision 檔案逐一比對。

因此它是可信的 vendor/source 起點，但仍要在 Lesson 3 進行人工型號、joint、TF、mesh 與尺寸核對。

## 你可以逐步親手重現

以下每一步都可以單獨執行。請看完該步的預期結果，再進下一步。

### Step 1：進入正確 workspace

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
pwd
```

目的：後續所有相對路徑都以 bundle root 為基準。  
成功畫面：最後一行應完整顯示上述 `robot129_pro6000_sim_20260913` 路徑。

### Step 2：確認搬移內容沒有損壞

```bash
bash tools/verify_bundle.sh
```

目的：逐檔比對 SHA-256，並阻止外部 symlink 或明顯 secret 混進 bundle。  
成功畫面：許多 `OK`，最後一行是 `PASS: ... all checksums match`。  
若看到 `FAILED` 或 `MISSING`，先停止，不要開始下一課。

### Step 3：重新擷取 server 基線

```bash
bash tools/collect_server_info.sh
```

目的：把當下硬體與工具狀態重新寫入環境報告。  
成功畫面：`Wrote .../environment/server_environment.txt`。  
這個腳本不會列出完整 environment variables，也不會收集 API keys。

### Step 4：閱讀 server 報告

```bash
less environment/server_environment.txt
```

目的：學會從證據確認 CPU、GPU、VRAM、driver、磁碟與 ROS 狀態。  
操作：方向鍵捲動，按 `q` 離開。  
重點：找到兩行 GPU、`docker daemon UNAVAILABLE`、`ros2 UNAVAILABLE`。

### Step 5：看 GPU 現在能不能使用

```bash
nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu --format=csv
```

目的：任何 Isaac Sim／vLLM 啟動前都先確認共享 GPU 負載。  
本課觀察：兩張 GPU 約 79% utilization，因此沒有啟動 Isaac Sim。  
這一步只讀取狀態，不會占用 GPU。

### Step 6：確認 Isaac 版本，但不啟動模擬器

```bash
/mnt/HDD4/wyattsheu/IsaacLab/.venv/bin/python -c "import importlib.metadata as m; print('Isaac Sim', m.version('isaacsim'))"
```

目的：從已安裝 package metadata 確認版本。  
成功畫面：`Isaac Sim 6.0.1.0`。  
這不是 GUI 啟動命令，不會載入場景。

### Step 7：進入研究環境

```bash
source /mnt/HDD4/wyattsheu/env_robot129_research/bin/activate
python --version
```

目的：讓離線研究程式不污染 Isaac Sim 環境。  
成功畫面：shell prompt 通常出現 `env_robot129_research`，Python 顯示 3.12.3。  
`source` 只影響目前 terminal；關掉 terminal 就解除。

### Step 8：跑離線單元測試

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/research
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python -m unittest discover -s tests -v
```

目的：檢查 SceneBundle、座標轉換、depth lifting、schema、overlay、replay 與 backend 安全邏輯。  
成功畫面：最後是 `Ran 108 tests` 和 `OK`。

### Step 9：跑合成資料 acceptance

```bash
TMPDIR="$PWD/.tmp" PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python scripts/validate_offline_pipeline.py
```

目的：用合成 RGB-D 與 replay 串起 grounding、lifting、evaluation 和可視化。  
成功畫面：命令最後印出一個新的 `out/validation/<UTC timestamp>` 路徑。

### Step 10：觀看結果圖

在 Codex／VS Code 檔案瀏覽器開啟剛才路徑內的：

```text
ab/overlay.png
```

預期畫面：

- 灰色背景；
- 左側紅色矩形；
- 右側綠色矩形；
- 紅色 GRASP 點、藍色 WAYPOINT、綠色 RELEASE 點；
- 下方有三種顏色的圖例。

若你是純 SSH terminal，不要為了看圖開外部 HTTP service；用 VS Code Remote 或下載單一 PNG 到本機查看。

## 本課修改的檔案

- `tools/collect_server_info.sh`：補上 CPU、GLIBC、uv、Apptainer／Singularity、NVIDIA container CLI 與 Docker daemon 權限狀態。
- `environment/server_environment.txt`：實際 server inventory。
- `environment/version_matrix.yaml`：以本機 Isaac Sim 6.0.1.0 取代 6.1.0 placeholder，選定未來 ROS 2 Jazzy 路線。
- `docs/progress/lesson_01.md`：本教學與驗收記錄。
- `research/out/validation/20260913T150415753964Z/`：離線 acceptance 產物。
- `/mnt/HDD4/wyattsheu/env_robot129_research/`：bundle 外的獨立研究環境。

## Done

- Bundle checksum、必要檔案、symlink 與 obvious-secret 檢查。
- 完整 server hardware／runtime inventory。
- 既有 Isaac Lab／Isaac Sim／Python 路徑與版本凍結。
- 官方 Isaac／ROS／MoveIt 相容路線查核。
- 獨立研究環境建立。
- 108 個離線單元測試。
- Synthetic replay acceptance 與 overlay 產物。
- 磁碟預算記錄。

## Verified

- Bundle 驗證 PASS。
- Isaac Sim 6.0.1.0 package metadata 與 lockfile 一致。
- Isaac Lab source `VERSION` 為 3.0.0。
- 既有紀錄顯示該環境曾成功產生 viewer 畫面／連線訊號。
- 離線 tests 108/108 PASS。
- Acceptance status PASS，model calls 0，robot commands 0。
- Piper URDF 與 STL 都存在且符合 bundle SHA-256。

## NOT RUN／UNVERIFIED

- Isaac Sim 本課啟動：NOT RUN，因兩張 GPU 都忙碌。
- ROS 2 Jazzy 安裝／啟動：NOT RUN。
- ROS domain 129 與 namespace `/robot129_sim` 隔離：UNVERIFIED。
- MoveIt／MoveIt Task Constructor：NOT RUN。
- Piper URDF 顯示、TF tree、joint direction 與實機 revision：UNVERIFIED，留到 Lesson 3。
- Piper USD 匯入、collision、physics、controller 與模擬運動：NOT RUN。
- VLM／vLLM 模型下載與推論：NOT RUN。
- Thor、CAN、Piper SDK、RealSense USB 與 production ROS domain：NOT CONNECTED。

## 主要產物

- `environment/server_environment.txt`
- `environment/version_matrix.yaml`
- `research/out/validation/20260913T150415753964Z/acceptance.json`
- `research/out/validation/20260913T150415753964Z/evaluation.json`
- `research/out/validation/20260913T150415753964Z/ab/overlay.png`

## 官方證據

- Isaac Sim 6.0.1 requirements：<https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html>
- Isaac Sim 6.0 Python environment：<https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_python.html>
- Isaac Lab 3.0 installation：<https://isaac-sim.github.io/IsaacLab/v3.0.0-beta2/source/setup/installation/index.html>
- Isaac Sim 6.0 ROS 2 configuration：<https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_ros.html>
- ROS 2 Jazzy Ubuntu target：<https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html>
- MoveIt 2 binary compatibility：<https://moveit.ai/install-moveit2/binary/>
- AgileX official Piper ROS repository：<https://github.com/agilexrobotics/piper_ros>
- AgileX official arm URDF repository：<https://github.com/agilexrobotics/agx_arm_urdf>

## 下一課

Lesson 2 只用 Isaac Sim 官方示例機器人理解 stage、prim、articulation、joint drive、timeline 與 reset。開始前先確認 GPU 空閒，並建立不會 `pkill` 其他使用者／同帳號程序的安全啟動方式。Lesson 2 尚未開始，等待 Wyatt 確認。
