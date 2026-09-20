# 開發指南：論文核心程式碼地圖 + Dashboard / WebRTC / 場景切換開發方向

寫給：Wyatt（要開始做更深入的開發工作，只想碰論文核心邏輯，不想深入 ROS/硬體細節）
用途：(1) 告訴你哪些檔案是論文相關、哪些是純基礎設施只要會呼叫就好 (2) 三個新開發項目（web dashboard、WebRTC 穩定化、場景/物體切換規範）的架構提案，還沒開始寫

---

## 1. 檔案地圖：論文核心 vs 基礎設施

### 1.1 論文核心（你應該花時間讀懂的）

這些是 ZeroDex §3.1/§3.4 對應的邏輯：VLM 語意定位 → affordance 區域判斷 → 3D 反投影 → 抓取候選生成。全部是純 Python，不依賴 ROS，可以在你自己的電腦上直接 `import` 測試，不需要 Isaac 開著。

```
research/src/mpg/
├── schema.py            # 論文的資料結構：StepType(GRASP/WAYPOINT/RELEASE/...), Plan,
│                         # LocatedStep, AffordanceRegion, PointStatus — 先看這個檔案，
│                         # 是所有其他模組共用的型別定義
├── grounding.py          # ★★★ 核心中的核心
│                         #   run_stage_a()    -- §3.1 選參考視角、判斷 pick/tool-use、排原語序列
│                         #   run_stage_b()    -- §3.1 把每個原語定位成 2D 像素點
│                         #   derive_grasp_hint() -- 橋接 stage_b 找到的抓取點 -> affordance prompt（式 9）
│                         #   run_grasp_affordance() -- §3.4 問 VLM「這裡能不能抓」，回傳 bbox
├── affordance_region.py  # §3.4 bbox -> 3D 遮罩：affordance_bbox_to_pixel_mask(),
│                         #   object_points_and_task_region_mask()（物體遮罩 ∩ affordance bbox）
├── lifting.py             # 2D 像素 + 深度 -> 3D 點雲的反投影數學（pinhole 公式），
│                         #   deproject()（單點）、deproject_region()（整塊區域）、桌面 RANSAC 擬合
├── grasp_candidates.py   # §3.4 附錄 B.1 簡化版：3D 點雲 -> 平行夾爪候選姿態
│                         #   generate_grasp_candidates() -- 掃 yaw、篩寬度/桌面淨空/對稱接觸點
├── grasp_contract.py     # 候選姿態的 JSON 序列化格式（讀/寫 candidates.json，給 MTC C++ 節點吃）
├── viz.py                 # VLM 輸出的視覺化：render_plan_overlay() 把 stage_a/b 的點畫成彩色圓點+
│                         #   標籤疊在原圖上，side_by_side() 左右對照 — dashboard 要顯示「VLM 框框」
│                         #   時，這裡的邏輯可以直接重用或參考
├── scene_bundle.py        # 讀取一次「擷取」的 RGB+深度+相機參數+TF，包成一個物件方便下游用
└── vlm/
    ├── base.py             # 所有 VLM backend 的共同介面（query(image, prompt) -> text）
    ├── local.py            # ★ 目前實際在用的：LocalQwenBackend，打本地 vLLM (Qwen3-VL-8B)，免費
    └── gemini.py           # 付費 Gemini API backend，需要 GEMINI_API_KEY，目前沒在用

research/prompts/
├── stage_a.txt            # §3.1 的第一層 prompt：排計畫、分「握把」vs「作用端」
├── stage_b.txt            # §3.1 的第二層 prompt：把每個原語定位成像素點
└── grasp_affordance.txt   # §3.4 D.2 附錄原文 prompt：問「這個區域能不能抓」

research/tests/            # 每個模組都有對應測試，想確認你改動有沒有壞掉東西，
                            # 跑：cd research && PYTHONPATH=src python -m unittest discover -s tests -v
```

**建議閱讀順序**：`schema.py` → `grounding.py`（重點看 docstring）→ `affordance_region.py` → `grasp_candidates.py`。這四個檔案看完，論文對應的邏輯鏈就完整了。`viz.py` 跟 `lifting.py` 屬於支援模組，需要時再查。

**legacy，可忽略**：`research/src/mpg/candidates.py`（舊的 2D 網格取樣，功能已被 `grasp_candidates.py` 取代，還沒清掉是因為有 3 個舊呼叫端還在用它，不是論文邏輯的一部分）。

### 1.2 場景幾何配置（半論文半基礎設施）

```
research/configs/scene_geometry.json   # 方塊/鎚形物的尺寸、位置 —— 單一事實來源
```
你如果要換場景上的物體，這是第一個要看的檔案（詳見本文件第 4 節）。

### 1.3 基礎設施（不用深入讀，只要會呼叫）

| 想做什麼 | 指令 | 說明 |
|---|---|---|
| 啟動模擬（headless，不開直播，給批次測試用） | `bash tools/start_robot129_grasp_sim.sh --scene pick_place` | `--scene` 可選 `marker`/`pick_place`/`pick_place_hammer` |
| 啟動模擬（開 WebRTC 直播，給你看即時畫面用） | `bash tools/start_robot129_ros_webrtc.sh --scene pick_place` | 佔用 port 49100，跟 headless 版本不能同時開同一場景 |
| 看目前狀態 | `bash tools/status_robot129_grasp_sim.sh` 或 `status_robot129_ros_webrtc.sh` | |
| 關閉 | `bash tools/stop_robot129_grasp_sim.sh` / `stop_robot129_ros_webrtc.sh` / `stop_any_webrtc.sh`（任何 WebRTC，含別人的程式） | |
| 重置場景到初始狀態 | `ros2 service call /robot129_sim/reset_scene std_srvs/srv/Trigger` | |
| 產生一組抓取候選（GT 幾何，不用 VLM） | `python tools/generate_s2_cube_candidates.py <out.json>` | |
| 用 MTC 對候選規劃 | `ros2 launch robot129_tasks mtc_pick_place_sim.launch.py report_path:=... candidates_path:=... max_candidates:=8` | 這個節點規劃完就結束，不是常駐服務 |
| 執行規劃出來的動作 | `ros2 run robot129_sim_execution run_grasp_motion --plan <plan.json> --out-dir <dir>` | 送 adapter、驗收三關卡、寫 report.json |
| 一鍵跑完整條鏈路（reset→候選→規劃→執行） | `bash tools/run_grasp_motion_demo.sh [--candidates-path PATH] [--record] [--out-dir DIR]` | ★ 最常用，需要 live Isaac 已啟動 |
| 啟動本地 VLM server | `bash tools/start_robot129_vllm.sh` / 關閉 `stop_robot129_vllm.sh` | Qwen3-VL-8B，免費，需要時才開 |
| 擷取一張 RGB-D 快照存成 SceneBundle | `python research/scripts/capture_scene.py --best-effort <scene_id>` | 給 `s4_live_affordance_smoke_test.py` 之類的腳本吃 |

你不需要知道 `robot129_mtc_pick_place.cpp`、`adapter_node.py`、`robot_model.py`、SRDF/URDF 這些檔案裡面在做什麼——它們是 MoveIt/ROS 控制鏈的實作細節，介面就是上表的指令。真的要查時再看：

```
ros2_ws/src/robot129_tasks/src/robot129_mtc_pick_place.cpp     # MTC 規劃節點（候選迴圈、place 偏移搜尋）
ros2_ws/src/robot129_sim_execution/robot129_sim_execution/
  ├── adapter_node.py       # FollowJointTrajectory -> Isaac 的橋接
  └── run_grasp_motion.py   # 執行器：送 adapter、驗收關卡、寫 report.json
sim/scripts/run_robot129_ros_webrtc.py   # Isaac 場景本體（spawn 物體、發 topic、收指令）
```

---

## 2. 「虛影圖片」的誠實澄清

你記得的「動作生成過程中產生的虛影圖片」——查證後，這其實是 **RViz MarkerArray 候選視覺化**（`/robot129_sim/grasp_candidate_markers` topic，每個候選一個箭頭 + 文字標籤，選中的綠色、失敗的紅色），是 **3D、即時渲染在 RViz 裡的**，不是一張存下來的靜態圖片。而且這台機器沒有 GUI 顯示環境，這個視覺效果從來沒有被人眼確認過（只驗證過「訊息有正確送達」）。

`research/src/mpg/viz.py` 有的是另一種東西：VLM 定位點疊加圖（`render_plan_overlay`），畫的是彩色圓點+標籤，不是候選姿態的虛影。

這代表 dashboard 要做「動作生成視覺化」時，兩個選項都要從頭做，不是接一個現成的圖片檔案：
- 接 RViz 那個 MarkerArray topic，在網頁上用 Three.js 一類的東西重畫 3D 候選
- 或是寫新的 2D 渲染：把每個候選的夾爪姿態投影回相機影像，畫成半透明的夾爪外框疊在 RGB 上（這比較接近你說的「虛影」，也比較容易在網頁上做）

第二個選項比較務實，下面第 3 節會展開。

---

## 3. Web Dashboard 開發方向

### 3.1 資料來源盤點

| 要顯示的東西 | 資料從哪來 | 目前存在嗎 |
|---|---|---|
| 相機即時串流 | ROS topic `/robot129_sim/camera/color/image_raw`（+ `aligned_depth_to_color`） | ✅ 已發布，但只有 ROS 訂閱者拿得到，瀏覽器拿不到 |
| VLM 輸出點位/框框 | `run_stage_a/b`、`run_grasp_affordance` 的回傳值 + `viz.py` 的疊圖邏輯 | ✅ 邏輯有，但只在腳本裡跑一次性存檔，沒有即時推送機制 |
| 候選姿態視覺化 | MTC 規劃節點的 `candidate_attempts`（寫進 report.json）+ RViz MarkerArray topic | ⚠️ 資料有，但是規劃跑完才一次性寫檔，不是即時串流；3D 視覺化只存在 RViz |

### 3.2 建議架構

**不要**把 dashboard 塞進 ROS node 裡（Python ROS + Web server 混在一起會很痛苦，且這台機器裝新套件受限）。改成三層：

```
┌─────────────────┐     ┌──────────────────────┐     ┌─────────────────┐
│ 瀏覽器 (前端)     │◄───►│ bridge server (新寫)  │◄───►│ 既有 ROS 系統     │
│ 純 HTML/JS，      │ WS  │ FastAPI + rclpy，     │ ROS │ (topic/service)  │
│ 不用裝任何東西    │     │ 一個 process 常駐      │     │                  │
└─────────────────┘     └──────────────────────┘     └─────────────────┘
```

- **bridge server**（新增，建議放 `tools/dashboard_server.py`）：一個獨立的 ROS 2 node + FastAPI/websockets server 混合 process（rclpy 支援跟 asyncio 一起跑，用 `rclpy.spin_once` 在背景 thread）。職責：
  1. 訂閱 `camera/color/image_raw`，轉成 JPEG，用 WebSocket 推給前端（不用 WebRTC，直播用 MJPEG-over-WS 對這種內部監控用途夠用、實作簡單很多，不用碰 port 49100 那些坑）
  2. 提供一個 REST endpoint 觸發 `run_stage_a`/`run_grasp_affordance`（直接 import `research/src/mpg`，不用另開 process），把回傳的點位/框框資料轉成 JSON 推給前端，前端疊在同一張 canvas 上畫
  3. 訂閱 `/robot129_sim/grasp_candidate_markers`（MarkerArray），轉成 JSON 陣列（每個候選的位置+朝向+分數+chosen/rejected）推給前端
- **前端**：單一 HTML 檔案，canvas 疊圖（相機畫面 + VLM 點位/框框 + 候選姿態的 2D 投影半透明外框），WebSocket 接 bridge server。不需要框架，純 JS 就夠，符合你機器裝套件受限的情況。

### 3.3 「候選虛影」怎麼畫

MTC 規劃節點已經有每個候選的 6D 夾爪姿態（`candidate_attempts[i].grasp_pose`）。要疊在 2D 相機影像上：
1. 用 `research/src/mpg/lifting.py` 的相機內參 + 當下 TF，把候選姿態的夾爪關鍵點（指尖、pinch 點）投影回像素座標（跟反投影是同一組公式的反方向）
2. Canvas 上畫半透明線框（selected=綠色實線，rejected=紅色虛線），這就是你要的「虛影」效果，而且是 2D、瀏覽器原生就能畫，不需要引入 Three.js

### 3.4 這是新開發，不是接現成的

老實說：以上三塊（MJPEG 推流、VLM 結果即時化、候選虛影投影）**目前都不存在**，是要新寫的功能，不是把現成模組接起來而已。工作量大致是：
- MJPEG 推流：小（幾十行，訂閱 topic 轉 JPEG）
- VLM 結果即時化：中（要把 `run_stage_a`/`run_grasp_affordance` 從「一次性腳本」改成「可重複呼叫的服務」）
- 候選虛影投影：中大（新的投影數學 + 前端疊圖）

---

## 4. WebRTC 穩定化 + 一鍵測試腳本

### 4.1 目前不穩定的根因（這輪除錯已發現的）

1. **Port 49100 是全機共用資源**，不是這個專案獨占——你自己的 `parcel-forge` 工具也會用它，兩邊會互搶（今天實際發生過：你的 WebRTC 畫面卡在 parcel-forge 的舊場景）。`tools/stop_any_webrtc.sh` / `~/stop_webrtc.sh` 已經處理這個，但**啟動前**還是要記得先確認 port 是空的。
2. **headless（`start_robot129_grasp_sim.sh`）跟直播版（`start_robot129_ros_webrtc.sh`）是兩個不同的 process**，不能同時對同一場景跑——這是設計上的限制，不是 bug，因為兩者都要獨佔同一組 Isaac GPU 資源與 ROS topic 命名空間。
3. **`run_grasp_motion_demo.sh` 目前沒有整合 WebRTC 啟動**——它假設 Isaac 已經在跑（不管是 headless 還是直播模式都可以，因為它只透過 ROS topic/service 溝通，不在乎有沒有開直播）。

### 4.2 建議的一鍵測試腳本

新增 `tools/run_grasp_demo_live.sh`，職責：
```bash
#!/usr/bin/env bash
# 1. 確保 port 49100 是空的（呼叫 stop_any_webrtc.sh）
# 2. 確保沒有 headless 版本的 grasp sim 在跑（避免跟直播版衝突）
# 3. 啟動 start_robot129_ros_webrtc.sh --scene "${1:-pick_place}"，等到 READY
# 4. 印出 WebRTC 連線資訊（140.113.203.85:49100）
# 5. 呼叫既有的 tools/run_grasp_motion_demo.sh 執行完整鏈路
# 6. 執行完印出 report.json 路徑 + （若有做 dashboard）dashboard 網址
```
這樣你只要跑一行 `bash tools/run_grasp_demo_live.sh pick_place`，就能「開直播 + 跑一次完整取放 + 邊看邊執行」，符合你說的「執行 run_grasp 之類的指令後能開始動」。

**我還沒寫這個腳本**——上面是設計，確認你要這個行為（尤其是「先強制關掉現有 WebRTC 再重開」這步，會中斷你正在看的畫面）之後我再動手。

---

## 5. 場景／物體切換規範（新場景、雜亂實驗室、換測試物品）

### 5.1 現況：目前沒有「規範」，是寫死的 if/else

`sim/scripts/run_robot129_ros_webrtc.py` 目前用 `--scene marker|pick_place|pick_place_hammer` 三個寫死的分支決定要 spawn 什麼（第 253-305 行一帶），物體是**程式產生的幾何體**（方塊/長方體），不是載入 USD 資產檔案。沒有「放一個教室場景」「放一個雜亂實驗室」這種機制——地板就是一個平面 + 目前這幾種物體。

### 5.2 建議規範：場景清單 (scene manifest) JSON

比照這輪已經建立的 `scene_geometry.json` 單一事實來源模式，擴充成一個更通用的「場景清單」格式，新增 `--scene-manifest <path.json>` 參數（跟現有 `--scene` 分支並存，不砍掉舊行為）：

```jsonc
// research/configs/scenes/cluttered_lab_01.json（範例，尚未實作）
{
  "schema_version": "scene_manifest_v1",
  "background": {
    "type": "usd_stage",              // 或 "procedural_floor"（目前唯一支援的）
    "usd_path": "/abs/path/to/lab_interior.usda"   // 要另外準備教室/實驗室的 USD 資產
  },
  "objects": [
    {
      "id": "target_cube",              // 對應既有的 topic/prim 命名，保持相容
      "kind": "primitive_box",          // 或 "usd_asset"
      "size_m": [0.035, 0.035, 0.035],
      "xy_m": [0.32, 0.0],
      "color_rgb": [0.88, 0.08, 0.04],
      "mass_kg": 0.08,
      "graspable": true                 // 給候選生成器/task_region 用
    },
    {
      "id": "distractor_mug",
      "kind": "usd_asset",
      "usd_path": "/abs/path/to/mug.usd",
      "xy_m": [0.20, 0.15],
      "graspable": false                // 只是雜物，不是抓取目標
    }
  ]
}
```

**每次測試想換物品**：不用改程式碼，寫一個新的 manifest JSON（或用小腳本產生隨機擺放的 manifest），跑：
```bash
bash tools/start_robot129_grasp_sim.sh --scene-manifest research/configs/scenes/cluttered_lab_01.json
```

### 5.3 這也是新開發，工作量誠實評估

- **procedural 物體（方塊/長方體）改成讀 manifest**：中——現有程式碼已經是「每個物體一組尺寸/位置/顏色參數」，改成從 JSON 陣列讀取、迴圈 spawn，是重構不是重寫
- **背景場景換成教室/雜亂實驗室的 USD 資產**：這是最大的未知數——**你需要先準備 USD 場景資產檔案**（自己建模、下載素材、或用 Isaac 內建 asset library），程式這邊只負責載入，不負責生成場景美術內容
- **`usd_asset` 物體種類（比方塊複雜的雜物）**：中——需要處理碰撞體形狀不再是簡單方塊，`grasp_candidates.py` 目前的候選生成假設物體是軸對齊盒子附近的點雲，換成任意形狀的雜物可能需要調整篩選邏輯（詳見 `grasp_candidates.py` 的 `WIDTH_EXCEEDED`/`NON_ANTIPODAL_EDGE_CONTACT` 篩選假設）

**我還沒寫任何一行程式碼**——這節純粹是規範提案，需要你確認方向（尤其是 USD 資產怎麼來）才能開始動工。

---

## 6. 下一步：這三項要怎麼排序？

三項工作量差異很大（dashboard 中大、WebRTC 腳本小、場景 manifest 中，但場景 manifest 卡在「你要準備 USD 美術資產」這個外部依賴）。建議先做 WebRTC 一鍵腳本（最小、立即有用），再做 dashboard 的 MJPEG 推流部分（最快看到效果），VLM 疊圖跟候選虛影可以晚一點做。場景 manifest 的程式碼部分可以先動工，但美術資產的部分要看你手上有什麼可以用。
