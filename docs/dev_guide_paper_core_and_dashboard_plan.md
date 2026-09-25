# 開發指南：論文核心程式碼地圖 + Dashboard / WebRTC / 場景切換開發方向

寫給：Wyatt（要開始做更深入的開發工作，只想碰論文核心邏輯，不想深入 ROS/硬體細節）
用途：(1) 告訴你哪些檔案是論文相關、哪些是純基礎設施只要會呼叫就好 (2) 三個新開發項目（dashboard-lite、WebRTC 穩定化、場景/物體切換規範）的架構提案與目前實作進度

**這份文件會持續更新**——你說要一直看它，之後每輪開發進度都會寫回這裡，不是寫一次就丟著。

**這個 repo 現在也在 GitHub 上**：https://github.com/wyattsheu/vlm-grasp-arm-sim（private）。push 上去的範圍見 `README.md` 最後一節「2026-09-20：這個 repo 實際 push 到 GitHub 的範圍」——只有原始碼/文件，`out/`、build 產物、`deployment/`（含真實 Thor 主機連線資訊）都刻意排除。之後有新進度，`git add` 對應檔案、`git commit`、`git push` 就會同步上去，我不會自動幫你 push，除非你要求。

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
| 用實機程式跑一次任務（本地模型／Gemini） | `bash tools/run_real_stack_task.sh "grasp the red block on the table"`，Gemini 前面加 `VLM_BACKEND=gemini` | 本地模式會自動開、關兩個 vLLM；詳見 §5.4.4 |
| 重置場景到初始狀態 | `bash tools/reset_robot129_scene.sh`（等同 `ros2 service call /robot129_sim/reset_scene std_srvs/srv/Trigger`，但不用先進 ROS 環境） | 手臂回 HOME、物體回預設位置；手臂還在跑軌跡時會被拒絕（`BUSY`），等它停再按 |
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

## 3. Dashboard-lite — 已實作，非即時網頁版

你把需求簡化成「每次 run 記錄相機影片 + VLM 框框（含時間/物件等細節）+ 候選虛影，越簡單越好」——這比原本 §3 設計的即時網頁監控面板單純很多，所以直接做完了，不是提案。**沒有網頁、沒有 server、沒有 WebSocket**，就是幾張存檔的圖片。

### 3.1 新增的程式碼

```
research/src/mpg/lifting.py
  def project_point(point_xyz_cam, k_matrix) -> (x_px, y_px)
    # deproject() 的反函式：3D 相機座標點 -> 2D 像素，畫「虛影」要用的投影數學

research/src/mpg/viz.py
  def render_affordance_overlay(image, region, *, caption_lines=())
    # VLM affordance 框框疊圖：橘色 bbox + 中心點 + 下方文字條（時間/物件/instruction/status）
    # UNAVAILABLE 時不畫框（沒有東西可畫），但文字條會誠實列出原因
  def render_candidate_ghosts(image, candidates, *, k_matrix, t_camera_world, chosen_id=None, caption_lines=())
    # 這就是你說的「虛影」：把每個候選的夾爪開合軸投影回相機影像，畫成一條線
    # 綠色實線=選中、灰色虛線=可行但沒選、紅色虛線=被拒絕
    # 小物體從正上方看，候選間距通常只有幾個像素，所以線段長度做了「視覺誇大」
    # （保留真實中心點與角度，長度不代表真實開合寬度，caption 裡有講明），
    # 每個候選只在圖上標一個數字索引，完整 id/score/狀態列在下方文字條

research/scripts/render_run_dashboard.py   # 新腳本，把上面兩個疊圖函式串起來
  用法：
  python research/scripts/render_run_dashboard.py \
      --scene-bundle <擷取的 scene bundle 目錄，含 rgb.png/tf.json/camera_info.json> \
      --affordance-json <run_grasp_affordance 存的 AffordanceRegion JSON> \
      --candidates-json <grasp_contract 格式的候選 JSON> \
      --chosen-id <選中的 candidate_id> \
      --object-id <物件名稱，純標記用> \
      --out-dir <輸出資料夾>
  產出：vlm_overlay.png（VLM 框框+caption）、candidates_ghost.png（候選虛影+caption）、summary.json
  影片：若有錄影（--video 指向 video.mp4），直接複製一份進 out-dir，不重新編碼
```

### 3.2 已用真實資料驗證過

拿 S4 真實跑（腕上相機真實擷取 + 真實本地 VLM）留下的 `research/data/scenes/s4_live_wrist_capture_01`（rgb/tf/camera_info）+ `out/grasp_motion/s4_live_test/affordance_region.json` + `s4_live_candidates.json` 跑過一次，兩張圖都正確：affordance 疊圖的框框準確落在方塊上；候選虛影圖裡 8 個候選繞著方塊中心呈放射狀排列，選中的 G000 是綠色實線，其餘依 accepted/rejected 分別是灰色/紅色虛線，下方文字條列出每個候選的完整 id/score/狀態。173 個既有測試（含新增的 12 個 viz 測試 + 2 個 project_point 測試）全數通過。

### 3.3 一鍵版本——已實作＋對真實 VLM 驗證

`tools/run_grasp_dashboard.sh`：reset 場景 → 擷取腕上相機快照 → 真實 VLM 定位＋affordance＋候選生成 → 渲染兩張圖，一行指令：

```bash
bash tools/run_grasp_dashboard.sh [scene_id] [instruction] [object_id]
bash tools/run_grasp_dashboard.sh                    # 全部用預設值
```

自動處理 vLLM 生命週期：如果 8001 port 已經有 vLLM 在跑就直接借用、結束不關；如果沒有就自己啟動、結束自動關閉（用 `trap cleanup EXIT`，只有自己啟動的才會自己關——不然會誤關掉你原本就在用的）。選中的候選（畫綠色實線那個）自動抓 `rejection_reasons` 為空、分數最低（依專案慣例 lower is better）的第一個。

**這個腳本存在的直接原因**：手動照三個步驟一步步下指令時，真的發生過「vLLM 才剛開始載入，下一步就把它關掉」的 race——因為手動下指令沒有等待/依賴關係保證。包成一支腳本、用同一個 shell 流程跑完，這個問題就不會再發生。

**跟 `run_grasp_motion_demo.sh`（MTC 規劃執行）還是分開的兩支腳本**，故意沒有合併：這支腳本產生的候選只是拿去畫圖，不會真的餵給 MTC 規劃或執行；要看「VLM 定位的候選有沒有真的被機器人抓起來」，還是要另外把這支腳本的 `s4_live_candidates.json` 當 `--candidates-path` 傳給 `run_grasp_motion_demo.sh`（跟 S4 章節示範的手法一樣）。合併成單一鍵是可以做的下一步，如果你要每次都「VLM 定位 → 真的執行 → 錄影 → 疊圖」全部一次做完的話跟我說。

---

## 3-old. Web Dashboard 原始提案（即時網頁版，已被上面的簡化版取代，留著備查）

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

**已實作＋對 live Isaac 驗證，過程中抓到一個一直存在、沒人發現的真實 bug**：

第一次跑 `tools/run_grasp_demo_live.sh pick_place` 時，MTC 規劃 PASS，軌跡也 SUCCEEDED，但 `physical_grasp_success=false`、`task_success=false`——查下去發現 `/robot129_sim/objects/target_cube/pose` 這個 topic 根本不存在。追到根因：**`tools/start_robot129_ros_webrtc.sh` 這個腳本從來沒有接收／轉傳 `--scene` 參數**，不管你傳什麼給它都被忽略，永遠用 `run_robot129_ros_webrtc.py` 的預設值 `--scene marker`（沒有物理方塊、只有純視覺標記的那個最早期場景）。這個 bug 應該從 `pick_place` 場景被加進來那天就存在，只是沒人踩到——今天之前所有成功的物理取放（含這輪 B 類的三次驗證）全部是透過 `start_robot129_grasp_sim.sh`（headless 版本，args 是正確用 `"$@"` 轉傳的），沒有人用直播版本跑過完整的取放鏈路。已修正（讓腳本接受 `--scene`，預設值維持 `marker` 不變，不影響任何沒傳這個參數的既有呼叫），修正後重新測試：`status=PASS`，`task_success=true`，`lift_delta_m=0.0602`，`place_error_z_m≈1e-7`。

**緊接著又抓到第二個真實 bug，也修好了**：確認 `--scene` 有效後，你問「相機影片呢？」，實測發現直播模式下錄影完全沒作用——`/robot129_sim/recording` service 回報「已開始」，但 `manifest.json` 永遠卡在 `status=RECORDING`、`frame_count=0`，沒有 `video.mp4`。追到根因：`sim/scripts/run_robot129_ros_webrtc.py` 裡負責錄影擷取畫面的 `overview_camera`（一個獨立的離屏相機）**只在 `args.record_only`（headless 模式）才會被建立**，而 `viewport`（直播用的互動視窗）剛好在 `not args.record_only` 時才建立——兩者互斥，代表「開直播」跟「錄影」這兩個功能設計上就沒辦法同時運作，錄影 service 開了個寂寞，捕不到任何一幀。已修正成無條件建立 `overview_camera`（改動只有拿掉 if 判斷，其餘程式碼都已經有 null check，不影響 headless 模式行為）。重啟 Isaac 重測：`run_0005` 錄到 95 幀、9.5 秒、640x480 h264 `video.mp4`，抽一幀出來看畫面正確——手臂正夾著紅色方塊。**現在直播看畫面 + 同時錄影可以同時運作了。**

---

## 5. 場景／物體切換規範（新場景、雜亂實驗室、換測試物品）

### 5.1 現況：目前沒有「規範」，是寫死的 if/else

`sim/scripts/run_robot129_ros_webrtc.py` 目前用 `--scene marker|pick_place|pick_place_hammer` 三個寫死的分支決定要 spawn 什麼（第 253-305 行一帶），物體是**程式產生的幾何體**（方塊/長方體），不是載入 USD 資產檔案。沒有「放一個教室場景」「放一個雜亂實驗室」這種機制——地板就是一個平面 + 目前這幾種物體。

### 5.1b 更新：真的有可用的免費資產庫，不用自己建模（2026-09-20 查證）

上一版寫「你需要先準備 USD 場景資產檔案」，查證後發現太悲觀了——這台機器對 NVIDIA 官方的 Isaac Sim 資產 CDN（`omniverse-content-production.s3-us-west-2.amazonaws.com`）有網路，之前用過的痕跡留在 `~/.nvidia-omniverse` 跟 `/tmp/https/...` 快取裡。實際用 `curl -I` 逐一探測過，這些路徑都是真的（HTTP 200，不是猜的）：

```
Isaac/Environments/Simple_Warehouse/warehouse.usd     # 雜亂倉庫場景（貨架、箱子），最接近「雜亂實驗室」的現成場景
Isaac/Environments/Office/office.usd                  # 辦公室場景，較乾淨、偏「教室」的調性
Isaac/Props/Mounts/SeattleLabTable/table_instanceable.usd   # 一張真的「實驗室桌子」資產
Isaac/Props/YCB/Axis_Aligned/010_potted_meat_can.usd  # YCB 物件資料集，household/lab 常見雜物
Isaac/Props/YCB/Axis_Aligned/025_mug.usd
Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd
Isaac/Props/YCB/Axis_Aligned/035_power_drill.usd
Isaac/Props/YCB/Axis_Aligned/019_pitcher_base.usd
```

YCB 是機器人抓取研究的標準資料集（馬克杯、洋芋片盒、電鑽、水壺、罐頭……），檔名照 `0XX_物件名.usd` 這種規則命名，上面 5 個是我實際探測過存在的；完整清單有 60+ 個物件，其餘的等真的要用時再逐一探測（`curl -I` 一秒內就知道存不存在，不用猜）。這些是 NVIDIA 官方隨 Isaac Sim 發佈、供模擬使用的標準資產，不是你自己下載的來路不明檔案。

**建議的具體組合**：`SeattleLabTable`（桌子）當背景平面，取代目前的純平面地板；`Simple_Warehouse`（雜亂）或什麼都不放（乾淨背景，只留桌子）依你要多雜亂決定；桌上散幾個 YCB 物件當干擾物，其中一個當抓取目標；一個簡單的綠色 pad（沿用現有 `pick_place` 場景已經有的放置區標記）當放置處。

### 5.2 已實作＋對 live Isaac 驗證：`--scene-manifest`

`sim/scripts/run_robot129_ros_webrtc.py` 新增 `--scene-manifest <path.json>` 參數，只對 `--scene pick_place`/`pick_place_hammer` 生效，純粹「附加」——不改抓取目標、不改地板、不改放置點，manifest 裡的每個物體**預設** spawn 成 kinematic/static（不會掉落、不參與物理碰撞的動力學運算），純粹是視覺／干擾雜物層（2026-09-21 起可以用 `"physics": "dynamic"` 改成真的有重力，見 §5.3），刻意不去動 `grasp_candidates.py`／MTC 碰撞已經驗證過的 `table_z_m=0.0` 假設。

```jsonc
// research/configs/scenes/cluttered_desk_01.json（已提交，對 live Isaac 驗證過）
{
  "schema_version": "scene_manifest_v1",
  "objects": [
    {
      "id": "mug", "kind": "usd_asset",
      "usd_path": "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/Props/YCB/Axis_Aligned/025_mug.usd",
      "xy_m": [0.20, 0.15], "z_m": 0.05, "yaw_rad": 0.0
    },
    {
      "id": "cracker_box", "kind": "usd_asset",
      "usd_path": "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd",
      "xy_m": [0.15, -0.12], "z_m": 0.05, "yaw_rad": 0.4
    },
    {
      "id": "distractor_block", "kind": "primitive_box",
      "size_m": [0.04, 0.04, 0.04], "color_rgb": [0.3, 0.3, 0.35],
      "xy_m": [0.42, 0.12], "z_m": 0.02
    }
  ]
}
```

`usd_path` 用的是 https 直連，不是 `omniverse://` 協議——查了 IsaacLab 自己怎麼解析資產根目錄（`isaaclab/utils/assets.py` 的 `ISAAC_NUCLEUS_DIR`），沒設 `ISAACSIM_ASSET_ROOT` 環境變數時，它自己也是從 `apps/isaaclab.python.kit` 讀出同一個 S3 URL 當預設值——這台機器對這個網域真的有連線（前一版寫「查證過 HTTP 200」但沒接進程式碼，這版已經接上並實際 spawn 成功）。

**怎麼用**：
```bash
bash tools/start_robot129_ros_webrtc.sh --scene pick_place --scene-manifest "$(pwd)/research/configs/scenes/cluttered_desk_01.json"
# 或 headless：
bash tools/start_robot129_grasp_sim.sh --scene pick_place --scene-manifest "$(pwd)/research/configs/scenes/cluttered_desk_01.json"
```
（`start_robot129_grasp_sim.sh` 本來就用 `"$@"` 原樣轉傳，不用改；`start_robot129_ros_webrtc.sh` 這輪順便補上 `--scene-manifest` 轉傳，跟修 `--scene` 那個 bug 是同一個檔案。）

**驗證**：對 live Isaac 啟動，log 印出 `SCENE_MANIFEST spawned mug/cracker_box/distractor_block`；擷取一張腕上相機快照，畫面裡看得到跟純色方塊明顯不同的真實幾何（YCB mesh 的陰影／外形），不是我們自己畫的色塊；接著重跑一次 `tools/run_grasp_motion_demo.sh`（沒有 manifest 相關參數，跑的是原本的方塊抓取），確認雜物不會干擾核心抓取流程：`status=PASS, task_success=true`，跟沒有 manifest 時的數字一致。

**下一步（還沒做）**：
- `target_cube` 本身納入 manifest（現在還是獨立寫死的抓取目標，manifest 只負責加雜物）
- 桌子／教室背景 USD（`SeattleLabTable`、`Simple_Warehouse` 已查到路徑，還沒接——這個風險比雜物高，因為要確認桌面高度跟 `table_z_m=0.0` 對得上，貿然換背景可能讓抓取的參考平面跟著跑掉）
- `usd_asset` 若要變成「可以抓的干擾物」（不只是雜物），需要碰撞體不再是軸對齊盒子，`grasp_candidates.py` 的篩選邏輯要跟著調整

### 5.3 新手教學：怎麼編輯場景、存起來、下次再用（2026-09-21）

#### 5.3.1 先搞清楚一件事：WebRTC 畫面裡改的東西「不會」被存下來

WebRTC 只是把 Isaac 正在跑的那個場景「直播」給你看，你在畫面裡拖動物體，改的是**這一次執行中、記憶體裡**的狀態。Isaac 程序一關（`stop_robot129_ros_webrtc.sh`、當機、重開機），這些改動就全部消失，下次啟動又回到程式寫死的樣子。

真正能「存下來、下次選擇要用哪個場景」的東西只有一個：**scene manifest JSON 檔**。每個 JSON 檔 = 一個場景配置，啟動時用 `--scene-manifest <檔案>` 挑一個。想要多個場景，就存多個 JSON（例如 `cluttered_desk_01.json`、`cluttered_desk_02.json`）。

所以流程是：

```
編輯 JSON  ──(--scene-manifest)──►  Isaac 場景  ──(capture_scene_manifest 服務)──►  新的 JSON
   ▲                                                                              │
   └──────────────────────────── 下次啟動時選它 ◄──────────────────────────────────┘
```

有兩條路可以產生 JSON：**(A) 直接手改 JSON**（最可靠，建議新手先用這個），或 **(B) 讓 Isaac 把目前場景「拍照」寫回 JSON**（capture-back，見 5.3.4）。

#### 5.3.2 路線 A：手改 JSON（建議先從這裡開始）

1. 複製一份現成的當起點：
   ```bash
   cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
   cp research/configs/scenes/cluttered_desk_01.json research/configs/scenes/my_scene.json
   ```
2. 用編輯器打開 `my_scene.json`，改 `objects` 清單（欄位說明見 5.3.3）。
3. 啟動時指定它：
   ```bash
   bash tools/stop_any_webrtc.sh      # 先確保沒有舊的在跑，不然 start 會直接回報「已在執行」而不會載入新檔
   bash tools/start_robot129_ros_webrtc.sh --scene pick_place --scene-manifest "$(pwd)/research/configs/scenes/my_scene.json"
   ```
4. 看 WebRTC 畫面確認位置對不對，不對就回到步驟 2 改數字、重啟。

**座標怎麼看**：世界座標原點在手臂底座正下方，單位公尺。`x` 往手臂正前方，`y` 往手臂左邊，`z` 往上，地板 `z=0`。抓取目標方塊預設在 `(0.32, 0.0)`，綠色放置區在 `(0.27, -0.12)`——雜物盡量不要擺在這兩點 5 cm 以內，不然會擋到抓取／放置路徑。現有 `cluttered_desk_01.json` 的擺法（`x` 0.15–0.42、`y` −0.12–0.15）已驗證不影響核心抓取，照這個範圍擺最保險。

**注意**：manifest 的 `id` 會變成 prim 路徑 `/World/Clutter/<id>`，同一檔案內 `id` 不能重複。

#### 5.3.3 欄位說明（`scene_manifest_v1`）

| 欄位 | 適用 | 預設 | 意思 |
|---|---|---|---|
| `id` | 全部 | `clutter_<序號>` | 物體名稱，不可重複 |
| `kind` | 全部 | `primitive_box` | `primitive_box`（程式產生的方塊，免下載）或 `usd_asset`（載入 USD 模型檔） |
| `xy_m` | 全部 | `[0,0]` | 物體中心的 x、y（公尺） |
| `z_m` | 全部 | `0.05` | 物體中心高度。方塊放在地上 = 邊長的一半（4 cm 方塊填 `0.02`） |
| `yaw_rad` | 全部 | `0.0` | 繞垂直軸旋轉（弧度，`0.785` ≈ 45°） |
| `physics` | 全部 | `static` | `static`：固定不動、沒有重力；`dynamic`：有重力、會掉、會被推、會被夾 |
| `mass_kg` | `dynamic` | `0.05` | 質量（公斤），只有 `dynamic` 才用得到 |
| `size_m` | `primitive_box` | `[0.05,0.05,0.05]` | 長寬高（公尺） |
| `color_rgb` | `primitive_box` | `[0.5,0.5,0.5]` | 顏色，0–1 |
| `usd_path` | `usd_asset` | 必填 | 模型檔路徑／URL，可用清單見 §5.1b |
| `scale` | `usd_asset` | `[1,1,1]` | 縮放 |

範例——加一個會掉下來、可以被推的黃色小方塊：

```json
{"id": "yellow_block", "kind": "primitive_box", "physics": "dynamic", "mass_kg": 0.02,
 "size_m": [0.03, 0.03, 0.03], "color_rgb": [0.9, 0.7, 0.1],
 "xy_m": [0.30, 0.20], "z_m": 0.10}
```

#### 5.3.4 「要怎麼有物理性質？」——`physics` 欄位

**只是「載入模型」不會自動有物理**。在這個專案裡，一個物體要不要受重力、會不會被推動，是由 spawn 時套的 `RigidBodyPropertiesCfg` 決定的，manifest 用 `physics` 欄位控制：

- `"static"`（預設）：`kinematic_enabled=True, disable_gravity=True`。有碰撞形狀（手臂撞到會被擋），但物體本身永遠釘在原地，不會掉也推不動。適合當背景雜物。
- `"dynamic"`：`kinematic_enabled=False, disable_gravity=False`，再加上 `MassPropertiesCfg(mass=mass_kg)`。跟抓取目標 `target_cube` 同一種設定——會掉落、會被撞倒、可以被夾起來。

**已實測**（2026-09-21）：一個 `dynamic`、3 cm 的方塊從 `z_m: 0.10` 生成，約 8 秒後 capture 讀到 `z = 0.015`——剛好是邊長的一半，代表它真的掉下來並停在地板上。

⚠️ `z_m` 設太低（物體一半插進地板）的 `dynamic` 物體會在第一個 physics step 被彈飛。不確定高度就設高一點讓它自己掉下來。

⚠️ `usd_asset` 設成 `dynamic` 可以生成，但**會讓 capture 失真**（見 5.3.6），而且 YCB 模型的碰撞形狀沒有特別驗證過，目前建議 `usd_asset` 維持 `static`。

#### 5.3.5 路線 B：把 Isaac 目前的場景「拍照」寫回 JSON（capture-back）

Isaac 在跑的時候，多了一個 ROS 服務 `/robot129_sim/capture_scene_manifest`：呼叫它，Isaac 會把每個 manifest 物體**現在**的位置／yaw 寫成一個新的 manifest JSON。用途：讓 `dynamic` 物體自己掉落、被手臂推過之後，把「落定後的樣子」存成下一次的起點。

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
"$micro" run -p /mnt/HDD4/wyattsheu/env_robot129_ros bash -lc '
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 service call /robot129_sim/capture_scene_manifest std_srvs/srv/Trigger "{}"
'
# 成功回應：success=True, message='captured 4 object(s) -> .../research/configs/scenes/captured_manifest.json'
```

- 預設寫到 `research/configs/scenes/captured_manifest.json`，**每次呼叫都會覆蓋**。要留下來就立刻改名：`mv research/configs/scenes/captured_manifest.json research/configs/scenes/my_scene_v2.json`
- 輸出路徑也可以改 ROS 參數 `capture.output_path`，但參數掛在節點 `/robot129_sim/isaac_articulation_bridge` 上（`ros2 param set /robot129_sim/isaac_articulation_bridge capture.output_path /abs/path.json`）；直接改名比較簡單。
- 沒有用 `--scene-manifest` 啟動時呼叫，會回 `success=False`，不會寫檔。
- 寫出來的檔案可以直接餵回 `--scene-manifest`——已實測「載入 → capture → 用 capture 出來的檔案重啟」整圈可以跑、位置對得上（包含 cracker_box 的 `yaw_rad: 0.4`）。
- 檔案裡的 `_provenance` 欄位記錄是在哪個 sim time 拍的，Isaac 載入時會忽略它。

**關於「在 WebRTC 畫面裡拖物體」**：capture 讀的是 PhysX 模擬中的即時位置，所以任何讓 `dynamic` 方塊移動的方式（重力、手臂推、Isaac 視窗裡 Shift+滑鼠拖曳這種物理拖曳）拍下來都會反映出來。但**在這個 WebRTC 直播介面裡實際用滑鼠拖曳，我沒有測過**——`static` 物體本來就拖不動（kinematic），`dynamic` 物體理論上可以用物理拖曳，但請自己試一次再依賴它。可靠的路線還是 A（改 JSON）。

#### 5.3.6 已知限制（誠實記錄）

**`usd_asset` 物體的 capture 不是即時位置，而是它當初被生成的位置。**

原因是除錯時發現的 Isaac Sim 原生 crash（沒有 Python traceback，log 只剩 `omni.hydratexture.plugin was already released` + `Unexpected reference count of 2 for UsdStage`）。用 bisection 逐一排除後，觸發條件是：**對 `usd_asset`（引用外部 USD 檔、例如 YCB 馬克杯）的 IsaacLab `RigidObject` Python handle「碰第二次」**——不論是把 spawn 時的 handle 保存下來跨過 `sim.reset()`（啟動約 10 秒就 crash），還是事後用 `RigidObject(RigidObjectCfg(prim_path=..., spawn=None))` 重新包一次（呼叫 capture 當下 crash）。`primitive_box` 兩種做法都沒問題。

另一條路「直接讀 USD stage 上的 transform」也試過，不能用：PhysX 不會每一幀把模擬結果寫回 USD 屬性，讀到的永遠是生成位置（而且那次連 cracker_box 的 yaw 0.4 都讀成 0）。

所以目前的行為是：
- `primitive_box`：capture 讀**即時**位置（先呼叫 `update()` 刷新快取再讀，不然會拿到舊值——這也是除錯中發現的第二個坑）。
- `usd_asset`：capture 原樣寫回**生成時**的 `xy_m/z_m/yaw_rad`。因為 `usd_asset` 預設 `static` 本來就不會動，這在預設情況下沒有誤差；只有你把 `usd_asset` 設成 `dynamic` 並讓它移動過，capture 才會跟實際位置不符。

這是 Isaac Sim 內部的問題，不是這支腳本能修的；程式碼裡 `spawn_clutter_from_manifest()` 的 docstring 有同樣的完整說明。

#### 5.3.7 常見狀況

| 症狀 | 原因／處理 |
|---|---|
| 改了 JSON，重啟後畫面沒變 | 舊的 Isaac 還在跑，`start` 只回報「已在執行」沒有重新載入。先 `bash tools/stop_any_webrtc.sh` |
| log 有 `Could not perform 'modify_rigid_body_properties' on any prims under: '/World/Clutter/mug'` | `usd_asset` 每次都會出現，是無害警告，正常啟動的 log 裡也有 |
| 第一次用某個 YCB 模型啟動很慢 | 正在從 NVIDIA CDN 下載，之後會用 `/tmp/https/...` 的快取 |
| `SCENE_MANIFEST skipping <id>: unknown kind` | `kind` 拼錯，只接受 `primitive_box` / `usd_asset` |
| `dynamic` 物體一開始就飛走 | `z_m` 太低、插進地板或跟別的物體重疊，調高 `z_m` |

2026-09-23 修正：manifest 物體過去一律**上下顛倒**生成（`InitialStateCfg.rot` 在這版 IsaacLab 是 `(x,y,z,w)`，舊的 helper 給的是 `(w,x,y,z)`，yaw=0 變成繞 X 轉 180°）。方塊看不出來，YCB 馬克杯是倒著的；已修，capture 讀回的 yaw 也一起改對。

### 5.4 實機架構場景：`pick_place_counter` + 真實 `mm_actions` 程式（2026-09-23）

目標：抓取方式完全照 `references/upstream`（實機程式）走，之後移植到實機時是**同一份程式碼**，不是「模擬版另寫一套」。

#### 5.4.1 手臂型號與模型

- 手臂是 **AgileX（松靈）PiPER**。NVIDIA 官方資產庫（Isaac 4.5–6.1，逐一掃過約 2.6 萬個檔案）**沒有 PiPER**，只有同廠的 LIMO 底盤；IsaacLab 也沒有 PiPER 設定。
- 我們現在用的 `sim/assets/robot129/robot129/robot129.usda` 本來就是 AgileX 官方 `piper_description` URDF + STL（MIT）用 Isaac 自己的 URDF importer 轉的（`sim/assets/robot129/import_report.json`），不是自己建的模型，不需要換。
- 實機用的 kinematics 是實驗室 fork 的 `roboticstoolbox`（`Tsaimingchun14/robotics-toolbox-python@8d8c0c3`，`models/URDF/Piper.py`），關節 origin 跟我們的 URDF 完全相同；在 OBSERVE 姿態兩邊 FK 都是 `(0.143, 0, 0.289)`。

#### 5.4.2 相機：過去的模擬相機方向是錯的

- URDF 的 `camera_mount` 讓光軸 = `gripper_base +X`，**跟夾取方向（+Z）垂直**。以前「看起來朝正下方」只是因為 HOME 姿態的夾爪剛好水平朝前。程式裡原本說「相機跟夾爪同軸」的註解是錯的，已更正。
- 實機的手眼校正在 `references/upstream/.../actions/base_action.py` 的 `ee_T_cam`（EE = `piper_gripper_base + 0.12 m`）：相機在 `gripper_base` 座標 `(-0.071, 0.024, 0.023)`，**光軸幾乎就是夾取方向**（差約 2°，另有約 7° roll，所以畫面地平線微斜是真的）。
- 新旗標 `--wrist-camera-model auto|urdf_nominal|real_calib`：`real_calib` 用實機校正 + D435 類內參（640×480、fx=fy=615）。`auto` = 只有 `pick_place_counter` 用 `real_calib`，舊場景維持 `urdf_nominal`，已驗證過的舊工具畫面不變。

#### 5.4.3 為什麼要墊高物

相機改成朝夾取方向後，在實機的觀察姿態 `OBSERVE = [0, 0.2, -0.6, 0, 0.8, 0]`（`ed305_arm_pose.py`，也是 `grasp.py` 夾完回去的姿態）只往下看 19°、高度 0.386 m，看不到地板。所以 `pick_place_counter` 在手臂前面放一個 **25 cm 高的檯面**（x 0.30–0.62 m、寬 0.60 m），目標物跟綠色放置墊都放在檯面上。數字在 `research/configs/scene_geometry.json` 的 `counter`；目標改成直立的 4×4×8 cm 方柱（實機 `grasp.py` 假設物體約 6 cm 厚，3.5 cm 小方塊太薄）。這個場景的 HOME 就是 OBSERVE。

#### 5.4.4 怎麼跑

```bash
bash tools/start_robot129_ros_webrtc.sh --scene pick_place_counter
bash tools/run_real_stack_task.sh "grasp the red block on the table"                  # 本地模型（預設）
VLM_BACKEND=gemini bash tools/run_real_stack_task.sh "grasp the red block on the table"   # 遠端 Gemini
bash tools/run_real_stack_task.sh "place the block on the green pad" --skip-observe
bash tools/reset_robot129_scene.sh        # 方塊掉到奇怪地方：手臂回觀察姿態、方塊回檯面 (0.44, 0)
bash tools/stop_robot129_ros_webrtc.sh    # 關掉模擬器（釋放 GPU 與 port 49100）
```

**選本地模型還是遠端 Gemini**（`VLM_BACKEND`，跟實機 `scripts/run_grasp.sh` 同一個開關）

| | 本地 `local`（預設） | 遠端 `gemini` |
|---|---|---|
| 怎麼選 | 不用設，或 `VLM_BACKEND=local` | 指令前加 `VLM_BACKEND=gemini` |
| 模型 | 兩階段：stage1 `Qwen/Qwen3-VL-4B-Instruct` 只讀文字、決定動作跟目標名稱；stage2 `allenai/Molmo2-4B` 看圖指出像素點，找不到就回 None | `gemini-robotics-er-2-preview` 一次做完 |
| 實機程式 | `LocalPipelineClient`（`reasoning/local_pipeline_client.py`），未修改 | `GeminiRoboticsClient`，未修改 |
| 要什麼 | 本機 GPU 約 11 GB + 14 GB（兩張卡各放一個）；權重已在 `/mnt/HDD4/wyattsheu/models/robot129_vllm/huggingface`（沒有的話 `bash tools/download_robot129_vllm_models.sh legacy-mm`） | API key（自動讀 `~/.config/robot129_gemini.env`）、網路 |
| 速度（實測） | 開引擎約 1.5–2 分鐘；VLM 本身 1.1 s | 不用開引擎；VLM 3 s～30 s 以上（塞車時要重試） |
| 會失敗在哪 | GPU 被別人占滿時引擎起不來（見下） | Google 塞車回 503 |

實機預設就是 `local`（`mm_actions_node.py` 的 `os.getenv("VLM_BACKEND", "local")`），模擬器照做，所以**不設就是本地**。

**本地的兩個 vLLM：每次執行才開，跑完自動關**

- `run_real_stack_task.sh` 在 `local` 模式下會自己呼叫 `tools/start_robot129_local_engines.sh` 開兩個引擎，任務結束（成功、失敗、Ctrl-C 都一樣）就呼叫 `tools/stop_robot129_local_engines.sh` 關掉，閒置時**不占 GPU**。不用自己開關。
- 連續跑好幾次（例如 grasp 接 place）想省每次 1.5–2 分鐘的開機時間：`KEEP_LOCAL_ENGINES=1 bash tools/run_real_stack_task.sh "..."`，全部跑完**記得** `bash tools/stop_robot129_local_engines.sh`。
- Port 是 **8010 / 8012**，不是實機的 8000 / 8002：這台機器的 8000、8002 已被別的程式占用。實機 client 本來就讀 `STAGE1_BASE_URL` / `STAGE2_BASE_URL` 環境變數，腳本用這兩個變數指過去，程式碼不用改；served model name 跟實機完全一樣（client 每個請求都會帶）。
- 每個引擎自動放到當下空間最多的那張 GPU，依序啟動（DEVLOG 2026-08-28 記過同時 profiling 會撞 race）。記憶體額度 stage1 0.12、stage2 0.15（佔整張 96 GB 卡的比例），依實測用量抓：兩張卡長期被別的使用者各占約 73 GB，只剩 16–24 GB。
- 引擎起不來、錯誤是 `Free memory on device ... is less than desired GPU memory utilization`：GPU 被別人用掉了。`nvidia-smi` 看一下；可以用 `STAGE1_GPU_UTIL` / `STAGE2_GPU_UTIL` 調低一點，或改走 `VLM_BACKEND=gemini`。log 在 `out/local_engines_robot129/*.log`。
- 跟實機不同的地方（誠實記錄）：實機是 docker 容器、vLLM 0.19；這裡是 venv `env_robot129_vllm`、vLLM 0.29。實機的完整啟動參數在 `new_modle_test/production_client/launch_local_engines.sh`，那個檔案不在任何 repo 裡，這裡的參數（`--max-model-len`、記憶體額度）是自己訂的。模型 revision 固定在 Qwen `ebb281ec`、Molmo2 `042abfa7`，實機用哪個 revision 未知。

**Gemini 回 503 怎麼辦**（2026-09-23 實際遇到）：錯誤訊息 `503 UNAVAILABLE ... This model is currently experiencing high demand` 是 Google 端預覽模型 `gemini-robotics-er-2-preview` 太忙，不是模擬器或程式壞掉。實機的 `GeminiRoboticsClient` 碰到例外會直接回 `None`，任務立刻以 `VLM_FAILED` 結束。模擬器外殼（不是實機程式）因此加了：

- **自動重試**（只有 gemini 預設開；local 預設不重試，因為 local 回 `None` 通常是 Molmo2 真的沒找到，溫度 0 重問只會一樣）：`decide_task` 回 `None` 時等 5 s、10 s、15 s…再問，預設最多多試 4 次，畫面印 `[VLM] RETRY`；`--vlm-retries N` 可調，`report.json` 的 `vlm_attempts` 記錄實際試了幾次。實測第 3 次成功，VLM 步驟約 31 s。
- **換模型**：`GEMINI_MODEL=gemini-2.5-flash bash tools/run_real_stack_task.sh "..."`，透過實機 client 本來就有的 `model=` 參數傳入；不設就是實機預設模型。換模型後 VLM 點位品質可能不同，結果不能直接跟預設模型比。
- 重試全部用完還是 `[VLM] FAILED`：等幾分鐘再跑，或換模型。前面沒有 503 訊息的 `NO_DECISION` 才是 VLM 真的看不到目標，要檢查 `vlm_input.png`。

- `sim/scripts/sim_mm_actions_node.py` 只重寫實機 `mm_actions_node.py` 的 ROS 外殼（實機版需要實驗室專用的 `mm_interface` 跟 `cv_bridge`），**grasp/place/Gemini client/本地 pipeline client/IK/servo 全部 import 自 `references/upstream`，一行都沒改、也沒複製進 repo**（`references/` 本來就不進 git）。
- 模擬器新增實機 driver 同樣介面：`/robot129_sim/piper/joint_cmd`（JointState，joint1–6 + `gripper` 開口 0–0.1 m，串流位置命令）、`/robot129_sim/piper/joint_states_feedback`。深度從 32FC1 公尺轉成 RealSense 的 16UC1 毫米再交給實機程式。
- 需要 venv `/mnt/HDD4/wyattsheu/env_robot129_realstack`（疊在 `env_robot129_ros` 上，裝實機 `requirements.txt` 的 pinned 版本：rtb fork、numpy 1.26.4、qpsolvers、quadprog、google-genai、rerun-sdk）。
- 每次輸出在 `out/grasp_motion/real_stack/<時間>/`：`vlm_input.png`、`vlm_debug.png`（VLM 點的位置）、`rerun.rrd`（實機程式本來就會 log 的 rerun 資料）、`report.json`（含模擬器的物體位置前後，**這是唯一能確認有沒有真的夾到的依據**）。

#### 5.4.5 實測結果（2026-09-23，Gemini robotics-er-2 與本地 Qwen+Molmo2）

| 動作 | 實機程式回報 | 模擬器真值 |
|---|---|---|
| grasp | 全部 stage SUCCESS | ✅ 真的夾到：兩指各約 7 N、開口 3.95 cm，物體從檯面 (0.44, 0, 0.29) 帶回 (0.13, 0, 0.31)，重跑一次結果相同 |
| grasp（本地 Qwen+Molmo2） | 全部 stage SUCCESS，VLM 1.06 s | ✅ 真的夾到：Molmo2 點在 (349, 190)，跟 Gemini 的 (346, 182) 差不到 10 px；物體帶回 (0.14, 0, 0.33)，抬高 3.6 cm。跑完兩個引擎自動關、GPU 用量回到原本 |
| place（修正前） | 全部 stage SUCCESS，`place complete` | ❌ 物體在 place 的 servo **一開始就掉到地上**，沒到綠墊 |
| place（修正後，`mm_system` 分支 `fix/place-hold-grip` b1c309d） | 全部 stage SUCCESS | ✅ 整段搬運都夾著、沒掉；⚠️ 但放下時**倒下**，躺在 (0.425, −0.087)，離綠墊中心 (0.44, −0.15) 約 6 cm |

**place 一開始就掉：原因與修正（2026-09-23，是實機程式的 bug，不是模擬器問題）**

- 原因：`base_action.move_arm_to_pose()` 每一步 servo 都送 `gripper = self._get_joint_state()[-1]`，也就是**量到的**夾爪開口。手上有東西時，量到的開口就是物體寬度 → 夾爪目標 = 目前位置 → 夾力歸零 → 物體滑出。
- 實機也會這樣：查了實機驅動 `nycu-acm/piper_ros`（commit 25c5d25，`piper_ctrl_single_node.py`），`joint_states_feedback` 的 gripper 是量測值（`GetArmGripperMsgs().gripper_state.grippers_angle`），收到命令後呼叫 `GripperCtrl(寬度, effort=1000, ...)`，是**位置命令＋力量上限**，所以實機一樣是「目標 = 現在位置、不出力」。
- 修正（`nycu-acm/mm_system` 分支 `fix/place-hold-grip`，2026-09-23 已 push、**未 merge 到 main**，上實機測過再開 PR；本 repo 的 `patches/mm_system/` 有 patch 備份與還原方法）：
  - `BaseAction` / `SlowBaseAction.move_arm_to_pose()` 加回 docstring 本來就有寫、但參數被拿掉的 `gripper_width=None`；`None` 維持原本行為（夾爪張開時用，例如 grasp 的接近段）。
  - `PlaceAction` 傳 `gripper_width=self._grasp_close_width`，也就是 grasp 閉合到、回 HOME 時一直鎖住的同一個寬度。
- 模擬器執行的是 `references/upstream/mm_system` **目前 checkout 的分支**；每次的 `report.json` 會寫 `mm_system: {branch, commit, dirty}`，看得出這次跑的是哪一版。切回 `main` 就會重現舊的掉落行為。

**放下時倒下（不處理，原 repo 也沒處理）**：實機 `place.py` 把末端送到「綠墊表面 + `PLACE_HEIGHT_OFFSET_M = 0.07` m」後直接張開。方塊 8 cm 高、夾在上半段，底部離墊子還有幾公分，張開後掉下去就倒了，往 +y 倒半個身長（4 cm）剛好解釋偏移量。實機放品客罐應該有同樣問題，要不要改 offset 或改成「往下降到接觸再放」待決定。

**抓取偵測（有沒有夾到）：原本的程式在哪裡**（使用者說過去有、被封印）

- `mm_system/main_ws/src/mm_actions/mm_actions/arduino_bridge_node.py`：從 Arduino（`/dev/ttyACM0`、115200）讀 5 路 PDMS 觸覺感測電壓，發 `/force_sensor_topic`（`Float32MultiArray`）。
- `mm_actions_node.py` 的 `is_holding_tightly()`：`use_force_grasp=true` 時看最後兩路電壓 ≥ 0.5 V；`false` 時只看「命令寬度 ≤ `grasp_close_width`」，**夾空也回 grasp complete**。
- `adaptive_grasping_node.py`（另一個獨立版本，提供 `/grasp`、`/release` service）：一步 0.5 閉合，任一路電壓比開始時掉超過 2.0 V 就停。訂的是 `/joint_state_feedback`（少一個 s），跟驅動發的 `joint_states_feedback` 對不上，看起來沒被用過。
- `main_ws/src/adaptive_grasping.py`（`gripper_force_test` node）：單獨測力覺用的小腳本，先記錄無接觸時的電壓 baseline，再一邊閉合一邊看最後兩路 ≥ 0.5 V；註解掉的另一種判斷是「比 baseline 多 0.5 V」。`mm_actions_node` 的判斷就是從這裡搬過去的。
- 被「封印」的方式：`start_mm_tmux.sh` / `run.sh` / `scripts/run_grasp.sh` 預設 `USE_FORCE_GRASP=false`，不啟動 `arduino_bridge_node`。原因是機器上目前沒裝感測器（GRASP_RUNBOOK §6）。
- 沒有感測器也能用的訊號：驅動的 `joint_states_feedback` 本來就有 `effort[6]`（夾爪力，`grippers_effort/1000`）和量測開口 `position[6]`。夾到東西時，量測開口會停在物體寬度、比命令寬度大（模擬器實測：命令 3.2 cm、量到 3.95 cm）；夾空時兩者會一致。這可以當新的抓取偵測依據，**尚未實作**。模擬器的 `/robot129_sim/piper/joint_states_feedback` 目前只發 position，`effort` 還沒補（Isaac 可以從手指接觸力或關節力算出來）。

同時重現了實機 DEVLOG 已知的問題：[ARM-01]「grasp complete 不代表夾到」、[ARM-09] IK 只約束位置、夾爪姿態自由。

---

## 6. 下一步：這三項要怎麼排序？

三項工作量差異很大（dashboard 中大、WebRTC 腳本小、場景 manifest 中，但場景 manifest 卡在「你要準備 USD 美術資產」這個外部依賴）。建議先做 WebRTC 一鍵腳本（最小、立即有用），再做 dashboard 的 MJPEG 推流部分（最快看到效果），VLM 疊圖跟候選虛影可以晚一點做。場景 manifest 的程式碼部分可以先動工，但美術資產的部分要看你手上有什麼可以用。

---

## 7. cuRobo 整合（6DoF grasp → 避障軌跡 + 即時避障）

**這節的範圍**：學長分工是「接收 6DoF grasp pose，用 cuRobo 做 IK/碰撞檢查/軌跡規劃，取代目前效果不好的運動生成」，另外加碼「後方立柱相機（只模擬相機本身，不模擬車台）」和「棒子即時揮動時手臂即時避開」。完整計畫見 `/mnt/HDD4/wyattsheu/.claude/plans/nvidia-rgb-https-github-com-nvlabs-gras-dapper-hopper.md`（Claude Code 的 plan 檔，不在這個 repo 裡）。這節只記錄**已經做過、可重現的**指令和狀態；還沒做的步驟標 NOT RUN，不要照抄成「已完成」。

### 7.0 新增的獨立環境（不動 IsaacLab venv）

cuRobo 裝在自己的 venv，跟現有的 `env_robot129_*` 系列平行，**不會**動到 `/mnt/HDD4/wyattsheu/IsaacLab/.venv`：

| 路徑 | 用途 |
|---|---|
| `/mnt/HDD4/wyattsheu/src/curobo` | cuRobo 原始碼，`git clone --branch v0.8.0 --depth 1 https://github.com/NVlabs/curobo.git`（4ea7736，2026-04-18 release） |
| `/mnt/HDD4/wyattsheu/env_robot129_curobo` | cuRobo 的 venv，Python 3.12，`torch==2.11.0+cu128`（跟 IsaacLab venv 同一個 torch 版本，不是巧合，是特地對齊的） |

**怎麼重建**（如果 venv 被刪掉或要在別台機器上重裝）：

```bash
cd /mnt/HDD4/wyattsheu/src/curobo
uv venv /mnt/HDD4/wyattsheu/env_robot129_curobo --python 3.12
VIRTUAL_ENV=/mnt/HDD4/wyattsheu/env_robot129_curobo uv pip install ".[cu12-torch,dev]" "torch==2.11.0" \
  --index-strategy unsafe-best-match --extra-index-url https://download.pytorch.org/whl/cu128
```

**要還原（解除安裝）**：`rm -rf /mnt/HDD4/wyattsheu/env_robot129_curobo /mnt/HDD4/wyattsheu/src/curobo`，不影響任何其他東西。磁碟用量約 5–8 GB。

**GPU 選擇**：這台機器兩張卡常被別人佔滿（2026-09-24 實測 GPU0 96% util / 89 GB used、GPU1 94% util / 83 GB used，只剩 8–15 GB 可用）。cuRobo 的指令都要帶 `CUDA_VISIBLE_DEVICES=1`（或視當下 `nvidia-smi` 結果選較閒的那張），不然會 OOM 或跟人搶。

**驗證過**（2026-09-24，都在 GPU1 上跑）：

```bash
P=/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python
export CUDA_VISIBLE_DEVICES=1
$P -c "import torch, curobo, warp; print(curobo.__version__, torch.__version__, warp.__version__)"
# -> 0.8.0 2.11.0+cu128 1.17.0，torch.cuda.get_device_capability(0) = (12, 0)（Blackwell sm_120，cuRobo 吃得下）

cd /mnt/HDD4/wyattsheu/src/curobo
$P -m curobo.examples.getting_started.motion_planning   # pose-to-pose + grasp 範例都 PASS，18.9s
$P -m curobo.examples.getting_started.reactive_control  # MPC 範例，position error 收斂到 0.0000，10.9s
```

**單元測試——⚠️ 讀這段再跑，不然會看到一堆假失敗**：

`pyproject.toml` 的 `addopts` 寫死 `-n 4 --dist loadscope`（4 個 xdist worker 平行跑）。在這台 GPU 被別人塞滿的機器上，4 個 worker 搶同一張卡會讓某些 worker 的 CUDA context 壞掉（`torch.AcceleratorError: CUDA error: an illegal memory access was encountered`），接著那個 worker 之後所有測試全部連帶失敗——**這是資源競爭造成的假失敗，不是 cuRobo 或這次安裝真的壞了**。

實測過程：`pytest --pyargs curobo.tests -q -p no:cacheprovider`（用預設 `-n 4`）→ `522 failed, 2786 passed, 39 skipped, 619 errors`（209s）。把失敗清單的前 60 個，加 `-o addopts= -n 0`（關掉 xdist、單一 process 序列跑）重跑 → `60 passed in 16.01s`，全部通過。這證實了假失敗理論；沒有把全部 522+619 個失敗都序列重驗（單一 process 跑完整 3963 個測試約需 15–20 分鐘，含 GPU 排隊時間可能更久），下次要「真的」驗證安裝完整性時用這個指令：

```bash
cd /mnt/HDD4/wyattsheu/src/curobo
CUDA_VISIBLE_DEVICES=1 /mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python \
  -m pytest --pyargs curobo.tests -q -p no:cacheprovider -o addopts= -n 0
```

如果 GPU 比較閒（`nvidia-smi` 兩張卡都 < 50%），也可以試 `-n 2 --dist loadscope`（兩個 worker、每個 worker 手動指定不同 GPU 比較難做到，xdist 不會自動幫你分卡，所以多 worker 還是有跟自己或別人搶同一張卡的風險，序列 `-n 0` 最保險）。

### 7.1 robot129 的 cuRobo 機器人設定：`research/configs/curobo/robot129.yml`

**怎麼重建**（從 URDF 擬合碰撞球，約 1–2 分鐘）：

```bash
cd /mnt/HDD4/wyattsheu/src/curobo
P=/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python
export CUDA_VISIBLE_DEVICES=1
URDF=/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/ros2_ws/src/robot129_description/urdf/robot129.urdf
# 注意：asset-path 直接指到 ros2_ws/src 就好，不用另外建 symlink——
# URDF 裡的 mesh 是 package://robot129_description/meshes/...，
# 而 ros2_ws/src/robot129_description/ 本來就存在，路徑天生對得上。
ASSET=/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/ros2_ws/src
OUT=/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/research/configs/curobo/robot129.yml
$P -m curobo.examples.getting_started.build_robot_model \
  --urdf "$URDF" --asset-path "$ASSET" --output "$OUT" \
  --clip-link base_link z 0.0 --sphere-density 3.0 --compute-metrics --export-xrdf
```

**⚠️ 第一次做的時候踩了一個坑，記在這裡避免重踩**：第一次是把 `--asset-path` 指到 `/tmp/claude-.../scratchpad/` 底下臨時建的 symlink，`RobotBuilder` 把這個路徑原封不動存進 `robot129.yml` 的 `asset_root_path` 欄位。這個 `/tmp` 目錄是 session-scoped，下一個 session 就會消失，等於 `robot129.yml` 之後會讀不到 mesh。修法：`--asset-path` 直接給 `ros2_ws/src`（上面已經改好），或者事後用 `yaml.safe_load` 讀出來改掉 `kinematics.asset_root_path` 欄位再存回去。**改這個檔案前先確認 `asset_root_path` 是不是指到 `/tmp` 或 `/mnt/HDD4/wyattsheu/handoff/.../ros2_ws/src`（後者才對）：**

```bash
/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python -c "
import yaml
d = yaml.safe_load(open('research/configs/curobo/robot129.yml'))
print(d['kinematics']['asset_root_path'])"
```

**碰撞球擬合品質（2026-09-24 實測，`--sphere-density 3.0`）**：

| link | 球數 | 覆蓋率 | 備註 |
|---|---|---|---|
| base_link | 8 | 61.9% | 固定底座，clip 在 z=0（车面高度） |
| link1 | 2 | 97.3% | |
| link2 / link3 | 17 / 17 | 88.5% / 94.0% | 大臂/小臂，長條形，擬合正常 |
| link4 | 1 | 97.8% | |
| link5 | 5 | 96.1% | |
| **link6** | **3** | **1.2%（很差）** | 手腕法蘭盤，只有 4mm 厚、直徑 3.5cm 的薄圓盤，球體天生擬合不好——已經試過 `--sphere-density 6.0 --protrusion-weight 40 --coverage-weight 2000` refit 單獨這個 link，改善有限。**已知限制，不是 bug**：這個 link 夾在 link5（96% 覆蓋）跟 gripper_base（96% 覆蓋）中間，兩邊都有夠大的碰撞球，實際上不太可能出現「只有 link6 那段撞到但兩邊球都沒偵測到」的情況，先接受這個品質，需要更精準時再回來調 |
| gripper_base | 4 | 96.3% | |
| link7 / link8（手指） | 6 / 5 | 91.5% / 96.5% | |

**手腕相機碰撞體——刻意沒做**：`AGENTS.md` 明講「camera 本體沒有碰撞幾何，不可亂猜實體尺寸」。沒有量測過的相機外殼尺寸就不編進 `robot129.yml`，維持現狀（只有機械臂本體有碰撞球，相機是視覺上跟著動但規劃器看不見它）。等有量測數據再補 `extra_links` + `extra_collision_spheres`。

**手動加的欄位**（`build_robot_model` 產生的預設值不對，事後用 `yaml.safe_load`/`yaml.safe_dump` 改的，不是靠 CLI 參數）：

```python
k = d['kinematics']
# pinch_center：MTC 真正用的 TCP（gripper_base + 0.125m 沿 local z），
# 不是 URDF 自己的 tcp link（0.180m，docs/progress/grasp_motion_interface_map.yaml
# 記過這個是錯的，指尖只到 ~0.136m）
k['extra_links']['pinch_center'] = {
    'joint_name': 'gripper_base_to_pinch_center', 'joint_type': 'FIXED',
    'link_name': 'pinch_center', 'parent_link_name': 'gripper_base',
    'fixed_transform': [0.0, 0.0, 0.125, 1.0, 0.0, 0.0, 0.0],  # [x,y,z,qw,qx,qy,qz]
}
k['tool_frames'] = ['pinch_center', 'camera_color_optical_frame']  # 兩個 URDF 裡本來就有的 frame link
k['cspace']['default_joint_position'] = [0.0, 1.2, -1.25, 0.0, 0.15, 0.0, 0.0345]  # SRDF home + gripper open
k['lock_joints']['joint7'] = 0.0345  # 夾爪開合不進 cuRobo 的 arm cspace，跟 MTC 一樣分開規劃/執行
```

`camera_mount`、`camera_link`、`camera_color_optical_frame`、`tcp`、`flange`、`tool0` 這幾個零質量 frame link **本來就在 URDF 裡**（`robot129.urdf:291-326`，都是 fixed joint），`build_robot_model` 的 URDF parser 會自動把它們留在運動鏈裡，不用額外用 `extra_links` 生出來——這是意外發現，一開始以為要自己補，後來確認 `tool_frames` 直接填 `camera_color_optical_frame` 就能查到 FK。

**FK 驗證過**（不是隨口說「應該對」，是真的兩條路徑算出來比對）：cuRobo 的 `Kinematics.compute_kinematics()` 在 SRDF home 姿態算出 `pinch_center = [0.395029366, -1.96e-6, 0.453423828]`，用 `yourdfpy` 直接讀同一份 URDF 獨立算一次（不透過 cuRobo）算出 `[0.395029392, -1.96e-6, 0.453423842]`——差在小數點第 7 位（< 0.001 mm），遠優於計畫要求的 1mm。四元數方向也對（差一個雙覆蓋的正負號，物理上是同一個旋轉）。指令：

```bash
P=/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python
export CUDA_VISIBLE_DEVICES=1
$P -c "
import torch
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import JointState
cfg = KinematicsCfg.from_robot_yaml_file('research/configs/curobo/robot129.yml')
robot = Kinematics(cfg)
q = torch.tensor([[0.0, 1.2, -1.25, 0.0, 0.15, 0.0]], device='cuda')  # 6 值，joint7 被 lock 了不算在 dof 裡
js = JointState.from_position(q, joint_names=robot.joint_names)
poses = robot.compute_kinematics(js).tool_poses.to_dict()
for name, p in poses.items():
    print(name, p.position.squeeze(0).tolist(), p.quaternion.squeeze(0).tolist())
"
```

**閉合軸方向——發現一個既有的疑似 bug，記錄但沒有動它**：機械手指（joint7/joint8）實際沿 `gripper_base`/`pinch_center` local **Y** 軸分開（用 `yourdfpy` 對 URDF 算 joint7 從 0 推到 0.035 時 link7 原點的位移方向，數值驗證過，見下）。但 `research/src/mpg/grasp_candidates.py` 的 `top_down_grasp_quaternion()` 把「閉合/開口寬度」評分的軸定義在 local **X**（`_closing_axis(yaw)` 的定義跟四元數建構方式互相印證，故意這樣寫，不是筆誤）。這兩個差 90 度。這是 cuRobo 整合過程中發現的，但 `grasp_candidates.py` 屬於 S3（候選生成）範疇，不是這次 cuRobo 整合的範圍，**這裡選擇不去動它、不靜默轉正**——只在 `research/src/mpg/curobo_bridge/frames.py` 的 module docstring 裡詳細記錄，之後誰接手 `grasp_candidates.py` 再處理。在對稱的方塊上看不出差別，鎚子握把（非對稱）才會踩到，而鎚子目前也還沒真的透過 MTC 執行過。

驗證閉合軸方向的指令（獨立於 frames.py 的常數，直接對 URDF 算兩次確認）：

```bash
P=/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python
$P -c "
import numpy as np, yourdfpy
urdf = yourdfpy.URDF.load('ros2_ws/src/robot129_description/urdf/robot129.urdf', load_meshes=False)
def link7_pos(j7):
    urdf.update_cfg({'joint1':np.float64(0),'joint2':np.float64(0),'joint3':np.float64(0),
                      'joint4':np.float64(0),'joint5':np.float64(0),'joint6':np.float64(0),
                      'joint7':np.float64(j7),'joint8':np.float64(-j7)})
    T_gb = urdf.get_transform(frame_to='gripper_base', frame_from='world')
    T_l7 = urdf.get_transform(frame_to='link7', frame_from='world')
    return (np.linalg.inv(T_gb) @ T_l7)[:3, 3]
print('displacement (0 -> 0.035):', link7_pos(0.035) - link7_pos(0.0))
"
# -> [0, -0.035, 0]：沿 gripper_base local -Y，不是 X
```

### 7.2 `research/src/mpg/curobo_bridge/` — 座標轉換橋接層

```
research/src/mpg/curobo_bridge/
├── __init__.py
└── frames.py   # grasp_candidate_v0 (xyzw) <-> cuRobo Pose (wxyz)；
                 # GraspGen-X 慣例（approach=+Z, closing=+X）-> robot129 pinch_center
                 # 慣例（closing=local Y，上面驗證過的真實閉合軸）的轉換
                 # 純 numpy，沒有 import curobo，兩個 venv 都能跑
```

`frames.py` 的 module docstring 就是上面 7.1 最後兩段的完整版，含推導過程，直接看檔案比看這裡更完整。

**測試**：`research/tests/test_curobo_frames.py`，9 個 test case。純 numpy 的部分（quaternion 轉換、GraspGen-X 轉換）兩個 venv 都能跑；牽涉 `yourdfpy` 的兩個 case（`test_finger_axis_matches_urdf`、`test_pinch_center_fk`，會重新對 URDF 算一次來跟寫死的常數/座標對帳）只在 `env_robot129_curobo` 執行，在 `env_robot129_research` 底下會顯示 `skipped`（不是失敗，是刻意 skip，因為 `env_robot129_research` 沒裝 `yourdfpy`）：

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/research

# 平常開發、快速迴圈：跟其他 167+ 個既有測試一起跑（2 個 curobo 相關的會 skip）
PYTHONPATH=src /mnt/HDD4/wyattsheu/env_robot129_research/bin/python -m unittest discover -s tests
# -> 2026-09-24 實測：Ran 182 tests ... OK (skipped=2)

# 要真的跑到 URDF 交叉驗證：換 curobo venv
PYTHONPATH=src CUDA_VISIBLE_DEVICES=1 /mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python \
  -m unittest tests.test_curobo_frames -v
# -> 2026-09-24 實測：Ran 9 tests ... OK（全部真的跑，沒有 skip）
```

**改 `robot129.urdf` 之後一定要重跑**這個指令（`env_robot129_curobo` 那條），因為 `test_finger_axis_matches_urdf` 和 `test_pinch_center_fk` 都是直接讀 URDF 算的，URDF 一改這兩個測試的「正確答案」也會跟著變，如果沒重跑會拿舊常數騙自己。

### 7.4 後方立柱相機：`--scene-camera pole`

**這節的範圍**：只模擬相機本身和它站立的細柱（不模擬車台/車子，照使用者指示）。相機的世界座標是估的，來自使用者提供的照片＋手機 AR 測距 App 草圖，使用者確認「大致對，先用推估值」——**不是量測值**，之後有真的量測數據要回來改 `research/configs/scene_camera.yaml`（唯一的事實來源，程式本身不寫死任何數字）。

**用法**：

```bash
bash tools/start_robot129_grasp_sim.sh --scene pick_place --scene-camera pole
# 或用 WebRTC 版本：tools/start_robot129_ros_webrtc.sh --scene ... --scene-camera pole
```

不加 `--scene-camera`（或明講 `--scene-camera off`）就是原本行為，完全不受影響——**已經實測驗證過**：default 模式下 `ros2 topic list | grep scene_camera` 是 0 筆，手腕相機照樣正常發布，log 沒有新的錯誤或警告。

**座標**（`research/configs/scene_camera.yaml` 的 `offset_from_base_m`，相對 `base_link` 原點＝world 原點）：behind 0.15m（x=−0.15）、手臂右側 0.115m（y=−0.115，REP103 慣例 −Y＝右）、高度 0.36m（z=0.36），水平朝 +X（跟手臂 joint1=0 時的伸展方向相同）。相機用 `Camera.set_world_poses_from_view(eyes=[offset], targets=[offset + (cosθ,0,-sinθ)])` 設姿態（θ = pitch，預設 0），不用自己手動湊四元數——這個做法直接借用 `overview_camera` 本來就在用的機制，讀出來的 `.data.quat_w_ros` 由 Isaac Lab 從實際的 prim transform 算，跟設定時用的方法無關。

**實測驗證過**（2026-09-24，`--scene pick_place --scene-camera pole`，HOME 姿態）：

```bash
# ROS 端檢查（先 source env_robot129_ros，見 §1.3 或 tools/robot129_ros_shell.sh）：
ros2 topic list | grep scene_camera
# -> .../scene_camera/color/image_raw, .../aligned_depth_to_color/image_raw, .../aligned_depth_to_color/camera_info
ros2 topic hz /robot129_sim/scene_camera/color/image_raw     # 實測 ~1.85 Hz（這台機器目前只有 ~0.16x 即時速度，GPU 被佔滿）
ros2 topic hz /robot129_sim/scene_camera/aligned_depth_to_color/image_raw   # 實測 ~3.4 Hz
```

存檔快照 `out/ros_webrtc_robot129/scene_camera/tf.json`：

```json
{"translation_xyz_m": [-0.15, -0.115, 0.36], "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5], ...}
```

位置精確等於設定值（沒有中間誤差），四元數 `(-0.5, 0.5, -0.5, 0.5)` 跟手算「水平看 +X、up=+Z、ROS optical 慣例」的四元數完全一致（獨立驗證兩次）。`depth.npy`：87957/307200 像素有限值（其餘是 clipping_range 5m 外或空景背景的 inf/nan），範圍 0.24–2.15m，合理。`camera_info.json` 的 fx=fy=855.17px，跟 `640*28/20.955` 手算的值一致。

**⚠️ 已知限制，實測確認、不是猜的**：HOME 姿態下這顆水平相機的畫面，左上角是手臂自己的夾爪（相機在肩膀後上方、水平看，夾爪剛好在視線附近），下方一小條是地板，中間一大片是空景背景色——跟計畫階段就預期的一樣：「水平相機在 0.36m 高度時，看不到 base 平面附近、離手臂很近的物體」。截圖存在 `out/ros_webrtc_robot129/scene_camera/rgb.png`（這是活動快照，會被下一次啟動覆蓋，不是永久證據檔）。`pick_place_counter` 場景（0.25m 高台面）或更遠處的物體應該在視野內，還沒實測，下次驗證時測一下。

**錄影**：`--record` 時現在有三個視角：`frames/`→`video.mp4`（overview，原本就有）、`wrist_frames/`→`wrist_video.mp4`（原本就有）、`scene_frames/`→`scene_video.mp4`（新的，只有 `--scene-camera pole` 時才會建立這個資料夾和輸出這支影片）。

**改了什麼（`sim/scripts/run_robot129_ros_webrtc.py`）**：新增 `--scene-camera {off,pole}` CLI 參數；新增 `SCENE_CAMERA_*` 常數（從 `scene_camera.yaml` 讀）；`publish_wrist_camera` 重構成呼叫共用的 `publish_camera_frame(camera, frame_id, ...)`，新增 `publish_scene_camera` 用同一個共用函式；`save_latest_wrist` 同樣重構成呼叫 `save_latest_camera`；錄影邏輯加第三個 `scene_frames` 分支。全部改動都用 `if SCENE_CAMERA_ENABLED` / `if scene_camera is not None` 包起來，wrist camera 的程式路徑沒有被改行為，只是換了個殼。

### 7.6 `plan_grasp.py` — cuRobo 抓取規劃器，接既有執行鏈

**這是目前最重要的一節**：cuRobo 規劃 → 既有的 `run_grasp_motion.py` / adapter 執行，**已經用真實 Isaac 跑通一次完整的 pick-and-place**（不是只有規劃成功，是方塊真的被夾起來、搬走、放下）。

**用法**（Isaac 要先啟動，`tools/start_robot129_grasp_sim.sh --scene pick_place`）：

```bash
bash tools/run_curobo_grasp_demo.sh
# 或指定候選檔／場景／後方相機柱子：
bash tools/run_curobo_grasp_demo.sh --candidates-path PATH --scene pick_place_counter --include-pole
```

跟 `tools/run_grasp_motion_demo.sh`（MTC 版）結構完全對應：reset → 產生候選 → 規劃 → 啟動 adapter → 執行，唯一差別是第 2 步換成 `research/src/mpg/curobo_bridge/plan_grasp.py`（cuRobo，`env_robot129_curobo`），不動 MoveIt/MTC 那一條路徑。輸出的 `plan.json` 跟 MTC 輸出的格式完全一樣（`sub_trajectories` 陣列），所以 `run_grasp_motion.py` 和 adapter 完全不用改。

**規劃流程**：`plan_grasp()` 呼叫 cuRobo 的 `MotionPlanner.plan_grasp()`（approach→grasp→lift 三段，一次呼叫，內建幫你挑最可達的候選），再用兩次 `plan_pose()` 做 transport（搬到放置點正上方）和 lower（降到放置高度），中間穿插三段夾爪開合。輸出 8 個 segment：ensure-open → approach → grasp → close → lift → transport → lower → release。

**已知簡化**（寫在 `plan_grasp.py` 的 module docstring，這裡摘要）：
- 目標物本身不在 cuRobo 的碰撞世界裡（只有地板／檯面／可選的立柱），lift 時沒有像 MTC 一樣把物體當 attached object。對我們這種短距離垂直的 top-down 抓取風險低，但雜亂場景會是真正的缺口。
- lift/place 的高度數學只對 top-down 候選成立（用 tool frame 的 z 偏移，top-down 時 tool +z＝world −z，所以負偏移才會往上抬）。
- 「place」只是兩次 `plan_pose` 加開爪，不是 MTC 那種有額外 Cartesian 約束的 lower/retreat。

**修一個安裝時遺留的設定檔問題**：`RobotBuilder.save()` 產生的 `robot129.yml` 帶了 `load_collision_spheres`、`num_envs` 兩個欄位，剛好跟 `RobotCfg.create()` 自己會傳的同名參數衝突（`TypeError: got multiple values for keyword argument`）。刪掉這兩個欄位（純粹是載入器的保留字，不影響碰撞球或環境數本身的實際行為，載入器有自己的預設值）。也把 `tool_frames` 從 `[pinch_center, camera_color_optical_frame]` 改成只留 `[pinch_center]`——cuRobo 的 IK/motion 規劃要求你對**每一個**列在 `tool_frames` 的 link 都給目標姿態，混進一個我們不打算約束的相機 frame 只會讓求解器多解一個沒人給值的目標。相機姿態要查的話用 `Kinematics.compute_kinematics()`/`get_link_poses()`，不需要它是 tool_frame。

**實測數字**（2026-09-23／24，`--scene pick_place`，方塊 `(0.32,0,0.0175)` → 放置點 `(0.27,-0.12)`，GPU 都在被別人佔用 93–96% 的狀況下跑）：

| | cuRobo | MTC（同一組候選，A/B 對照） |
|---|---|---|
| 規劃結果 | PASS，選中 G007 | PASS |
| 規劃時間 | 6.9–17.0s（`plan_grasp.py` 內部計時，會依 GPU 空檔波動） | 整條 reset→候選→規劃→執行 wall-clock 2m33s（沒有單獨量到純規劃時間，MTC 輸出沒有記這個欄位） |
| `physical_grasp_success` | true（link7/link8 真的量到接觸力 ~10N） | true |
| `lift_delta_m` | 0.0501–0.0501 | 0.0602 |
| 最終方塊位置 vs 目標 `(0.27,-0.12)` | `(0.2696,-0.1198)`，誤差 ~0.4mm | `(0.2678,-0.1190)`，誤差 ~2.2mm |
| `task_success` | true（4 次嘗試中 3 次一次到位；1 次卡在中途 tolerance） | true |

**⚠️ 誠實記錄一個間歇性失敗，不是每次都一次過**：跑了 4 次完整流程（1 次手動 + 3 次用一鍵腳本），其中 2 次從頭到尾 `task_success=true`；1 次只有最後「開爪放開」那步的 joint7 位置誤差 0.00325m 超過容忍值 0.003m（已經修：把夾爪開合動作的時長從 0.5s 拉到 1.0–1.2s，之後沒再重現這個特定失敗）；1 次是「approach」那段手臂到位時 joint5 誤差 0.0345 rad 超過 0.03 rad 容忍值，重跑同一個腳本（GPU loading 不同）就過了。這兩種失敗都是 `FollowJointTrajectory` 完成時的容忍值檢查，跟 GPU 被佔用導致的即時速度波動有關（這台機器目前只有約 0.16x 即時速度，且會變動），**不是 cuRobo 規劃本身的問題**——同一份 `plan.json` 重新執行過確實能成功。`adapter_params.yaml` 的 `arm_goal_tolerance_rad: 0.03` 這個值本身就是之前 MTC 也踩過同一類問題後才放寬的（見它自己的註解），這是這條共用執行鏈本來就有的已知特性，值得記錄但不屬於這次 cuRobo 整合要解決的範圍。

**指令**（單獨跑規劃，不執行）：

```bash
export CUDA_VISIBLE_DEVICES=1   # 或用 nvidia-smi 挑目前較閒的那張卡
/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python research/src/mpg/curobo_bridge/plan_grasp.py \
  --candidates out/grasp_motion/candidates/curobo_test_candidates.json \
  --scene pick_place \
  --out out/grasp_motion/curobo/test_run/plan.json
```

候選檔可以用既有的 `tools/generate_s2_cube_candidates.py`（已知方塊幾何，免 VLM）產生，跟 S4 的 live-VLM 候選檔格式完全相容（都是 `grasp_candidate_v0`）。

### 7.8 `reactive.py` — 即時避障（Mapper 融合深度 → ESDF → MPC），真的跑通了

**架構上跟計畫原稿不一樣，這裡先說明為什麼**：原計畫寫「在 Isaac 行程內 lockstep 執行」，實作時發現這需要把 cuRobo 裝進 IsaacLab 共用的 venv（這台機器上其他專案也在用，AGENTS.md 本來就禁止亂動）才能在同一個 Python process 裡 import，風險偏高又難在不影響 Isaac 的前提下單獨測試。改成**獨立的 ROS 2 node**（在 `env_robot129_curobo` 跑），訂閱既有的相機/關節話題，輸出發到既有的 `/robot129_sim/piper/joint_cmd`（原本就是給即時串流指令設計的介面：「no trajectory interpolation ... a streamed command supersedes any active plan」，MPC 的輸出剛好符合）。跟這個 repo 其他地方的架構慣例一致（Isaac／ROS／research venv 永遠是分開的 process，靠 ROS 或檔案溝通，從不共用 interpreter），代價是 ROS 傳輸延遲，換來的是不用動共用資源、之後要包成實機 node 也更直接（本來計畫就寫了這是下一步）。

**用法**（Isaac 要先用 `--scene dynamic_stick --scene-camera pole` 啟動）：

```bash
export CUDA_VISIBLE_DEVICES=1
/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python research/src/mpg/curobo_bridge/reactive.py \
  --include-pole --switch-period-s 12 --control-rate-hz 3 --duration-s 90
```

`--include-pole` 要跟 Isaac 那邊的 `--scene-camera pole` 搭配（cuRobo 的已知世界模型要跟 Isaac 實際擺的東西一致，不然會閃一個不存在的障礙物）。`--goal-a`/`--goal-b` 預設是 `pinch_center` 在 `(0.30, ±0.15, 0.35)` 交替，`--switch-period-s` 控制多久切換一次。

**開發過程中發現一個真的 cuRobo bug（在這裡修掉，沒有回報上游）**：`CameraObservation.depth_to_meter` 預設 `0.001`（假設你給的是 RealSense 常見的原始毫米深度），但這個模擬本來就發布公尺深度（`32FC1`）。沒設的話，深度值會被默默乘以 0.001，所有反投影出來的點會塌縮到相機鏡頭前幾毫米——這造成兩種完全相反、但都是錯的沉默失敗：手腕相機（離自己夾爪只有幾公分）算出 100% 像素都被判定是機械臂本體；後方相機（離手臂約 0.4m）算出 0%。花了不少時間才用「真的擷取一組同步的深度+關節角度」而不是隨便編一組合成資料，才抓到這個欄位。**這個模組裡每一個 `CameraObservation` 都明確帶 `depth_to_meter=1.0`**（`_camera_observation()` 這個共用函式），細節寫在 `reactive.py` 的 module docstring。

**另外兩個踩到的坑**（也都在 docstring 裡）：
- `RobotSegmenter.from_robot_file()` 的路徑是相對 cuRobo 自己打包的 `content/configs/robot/` 目錄解析，不是 CWD 也不是絕對路徑——要繞過去用 `RobotSegmenter(Kinematics(...), ops_dtype=torch.float32)` 直接建。
- `RobotSegmenter` 預設 `ops_dtype=torch.bfloat16` 會直接跟 `robot129.yml` 的 float32 碰撞球衝突噴 `TypeError`——一定要明確傳 `ops_dtype=torch.float32`。

**驗證過程**（2026-09-24，誠實記錄怎麼抓到上面的 bug，不是憑空寫對的）：
1. 先用合成資料（整張深度圖固定 0.6m）測 `RobotSegmenter`——結果整張圖 100% 被判為機械臂，明顯不對。
2. 換成真實擷取的手腕相機資料，但關節角度用「大概是 HOME」去湊——這次變成 100%（camera 太近，本來就有問題，不是好的測試案例）。
3. 寫一個小工具同時訂閱 depth/info/tf/joint_states，抓「真的同步」的一組資料（`research/tests/fixtures/scene_camera_home/`，現在也是單元測試的 fixture），改用後方相機（離手臂較遠，是比較有代表性的測試案例）——這次結果先是 0%（也不對），才回頭去查 `CameraObservation` 的欄位預設值，抓到 `depth_to_meter`。
4. 修好之後同一組同步資料重測：8.4% 像素被判為機械臂，遮罩的像素範圍剛好落在畫面左上角——跟 §7.4 那張 `rgb.png` 截圖裡夾爪出現的位置完全吻合。這時候才有信心繼續往下接 Mapper/ESDF/MPC。
5. 串通整條 segment→filter→integrate→compute_esdf→MPC 的 pipeline，用同一組真實資料跑一次，全部成功（`research/tests/test_reactive_controller.py` 現在就是這個回歸測試）。

**新場景 `--scene dynamic_stick`**：一根 0.03×0.03×0.4m 的直立木棒（kinematic body，有真的碰撞/接觸感測，但不受物理推動——姿態由公式驅動），中心在 `(0.30, 0, 0.35)`，沿世界 Y 軸正弦擺動（振幅 0.22m、週期 6s，數字是設計出來威脅 `--goal-a`/`--goal-b` 之間的直線路徑，不是量出來的）。**刻意不加進任何規劃器的已知幾何**（`plan_grasp.py`、`reactive.py` 都沒有這根棒子），只能靠即時深度看到——這是這個測試的重點。在 `link3`／`link4`／`link5`／`link6`／`gripper_base` 上都掛了 contact sensor，發布 `/robot129_sim/contacts/{link}`（跟 pick_place 場景的 `link7`/`link8` contact 用同一種機制，物理真值，不是幾何算出來的距離）。

**2026-09-24 展示調整**：上段是舊版測試條件及其歷史數據。現行預設改為中心 `(0.15, 0.35, 0.25)`（與靜態角錐同一條 A→B 路線）、左右各 0.015m、週期 12s，使木棒在原地緩慢晃動，並避開 HOME 夾爪。後方相機為展示取景移到 `(-0.40, -0.115, 0.60)`、向 A→B 障礙區轉向 30° 並下俯 25°，焦距設為 18mm；原始照片估計 `(-0.15, -0.115, 0.36)`、水平朝向另存於 `scene_camera.yaml`，均尚非實機校正值。下列舊版 297N 碰撞及 MPC 數據不可當作新預設的驗收結果。

**實測結果**（2026-09-24，`--scene dynamic_stick --scene-camera pole`，90 秒 wall-clock）：

| | 沒開避障控制器（基準組） | 開了 `reactive.py` |
|---|---|---|
| 手臂在 HOME、棒子在初始位置時的 `gripper_base` 接觸力 | **297N**（真的撞上了——棒子中心 X/Z 剛好跟 HOME 姿態下的 gripper_base 位置重疊，見下方說明） | 控制器啟動後立刻變成 0N（見下） |
| 手臂持續運動 32 秒內取樣 4 次的接觸力 | （未測，基準組手臂本來就不太動） | 全部 4 次、`gripper_base`／`link5`／`link6` 都是 **0N**，同時 joint1 從 −0.37 一路變化到 1.33 rad（證明手臂真的在動，不是卡住不動才沒碰到） |
| MPC 控制步數 / 求解失敗次數 / ESDF 更新次數 | — | **264 / 0 / 67**（264 次全部成功求出動作，沒有一次「沒有可用動作」） |

基準組的 297N 不是意外——棒子中心 `(0.30, 0, 0.35)` 跟 SRDF home 姿態下 `gripper_base` 的 FK 位置 `(0.27, 0, 0.455)` 距離很近（棒子 z 範圍 0.15–0.55 蓋住 0.455；x 只差 3cm，加上兩者實際碰撞體都有幾公分大小），棒子的初始姿態本來就卡在手臂正下方——這正好是個乾淨的「沒有避障=真的會撞上」對照組，不是刻意做壞的假象。

**這裡沒做、留給下次**：
- 沒有做「關掉 Mapper、只用已知幾何」的對照組（計畫裡提過的驗證項目之一）——上面基準組（控制器完全沒開）已經證明了棒子是真的威脅，判斷這個對照組的邊際價值較低，先跳過。
- 只測了後方相機（`--scene-camera pole`），沒有把手腕相機也加進 Mapper（module docstring 有說明：手腕相機離自己太近，分割後大部分視野可能還是自己的硬體，需要另外驗證才能加）。
- `--duration-s` 是量 wall-clock，不是 sim time（跟即時避障的即時性定義一致，但如果要精確對照「棒子擺了幾個週期」要注意這台機器目前只有 ~0.16x 即時速度）。
- 沒有測過真的更快、更難躲的棒子擺動（目前週期 6s 偏慢）。
- 各階段（segment/integrate/esdf/mpc）個別的 wall-clock 延遲沒有分開量測，只有整體的「264 步、90 秒」這個總體數字。

### 7.9 現在的狀態 / 還沒做的部分（NOT RUN）

| 項目 | 狀態 |
|---|---|
| cuRobo 安裝、範例、FK 驗證 | ✅ 完成，見 7.0/7.1 |
| `robot129.yml`、`frames.py`、單元測試 | ✅ 完成，見 7.1/7.2 |
| 後方立柱相機（模擬相機本體，不模擬車台） | ✅ 完成，見 7.4，已用真實 Isaac 啟動驗證過 |
| `plan_grasp.py` + `tools/run_curobo_grasp_demo.sh` | ✅ 完成，見 7.6，已用真實 Isaac 跑出至少 2 次完整成功的 pick-and-place，並和 MTC 做過一次 A/B 對照 |
| `reactive.py` + `--scene dynamic_stick` | ✅ 完成，見 7.8，用真實 Isaac 跑通 264 步 MPC、0 次碰撞、跟「沒開控制器就會撞」的基準組做過對照 |
| `pick_place_counter`／更遠場景下後方相機視野的實測 | ❌ NOT RUN（見 7.4 最後一段） |
| 大樣本（>4 次）重跑統計，量化 §7.6 提到的間歇性 tolerance 失敗發生率 | ❌ NOT RUN，只有 4 次的小樣本 |
| lift 時把目標物當 attached object（cuRobo 碰撞世界） | ❌ NOT RUN，見 7.6 已知簡化 |
| 手腕相機加入 `reactive.py` 的 Mapper | ❌ NOT RUN，見 7.8 |
| 「關掉 Mapper」對照組、各階段延遲分開量測 | ❌ NOT RUN，見 7.8 |
| 把 `reactive.py` 包成可以對實機跑的 ROS node | ❌ NOT RUN（架構上已經朝這個方向設計，但沒有實機可測） |

這四個 Step（cuRobo 安裝 → robot129 設定 → 後方相機 → plan_grasp.py → 即時避障）都做完並用真實 Isaac 驗證過了。下次要做的話，先看這張表裡還沒做的項目，回來更新這節。

---

## 8. 避障演示——四視窗 Rerun 介面（2026-09-24 開始）

計畫檔：`/mnt/HDD4/wyattsheu/.claude/plans/pasted-content-id-f92c-rgb-3d-stateful-pebble.md`（Claude Code 的 plan 檔，不在這個 repo 裡）。目標：一個可以現場操作、同時錄影的演示，四視窗（3D 障礙物偵測、全局視角、臂後相機、臂頂相機＝腕上相機），涵蓋揮動棍棒／靜態障礙物／A→B 避障／多種目標抓取／6 軸關節輸入五種情境。

### 8.1 Phase 0：A/B 示範姿態——已實作＋對真實 cuRobo IK 驗證，有真實發現

`research/scripts/select_demo_poses.py`：對指定的「approach 方向」（up/left/down/...）掃一片工作空間網格，用 cuRobo 的 collision-aware IK 逐一驗證可達性，而不是假設某個方向一定可達。

**真實發現**：一開始假設的工作空間（z≈0.35m，跟 `reactive.py` 原本 `--goal-a/--goal-b` 預設的 `(0.30, ±0.15, 0.35)` 同一個量級）裡，「夾爪朝上」幾乎不可達——joint5 只有 ±1.22 rad，掃到的唯二可行解都卡在離極限只剩 0.02–0.03 rad 的邊緣（`j5≈±1.20`），這種解在實際執行時很可能跟 §7.6 記錄的 tolerance 間歇性失敗一樣不穩定。把搜尋範圍往上擴到 z≈0.5–0.6m 後，找到舒適的解（joint5 餘裕 0.63–0.89 rad）。

最終選定（`research/configs/demo/ab_poses.yaml`，已提交，`--include-pole` 驗證過不會跟立柱碰撞）：

| 點 | 位置 (m) | 方向 | joint5 餘裕 | 獨立 FK 覆核誤差 |
|---|---|---|---|---|
| A | (0.25, 0.35, 0.58) | +Z（上） | 0.633 rad | 3.0° |
| B | (0.05, 0.35, 0.35) | +Y（左） | 0.889 rad | 3.4° |

「獨立 FK 覆核」：用 `research/src/mpg/urdf_fk.py`（跟 cuRobo 完全獨立的第二套 FK 實作，純 numpy）重算 IK 解出的關節值，確認 approach 軸方向跟宣告的一致——這不只是信任 cuRobo IK 回報的 residual error，是真的用另一套算法覆核。兩點的位置誤差 6.1mm / 9.6mm、角度誤差 3.4°/3.4°（收斂到 `--orientation-tolerance 0.08 rad` 內，不是精確命中）。

建議的錐形障礙物位置（A、B 中點）：`(0.15, 0.35, 0.465)`，已用於 `research/configs/scenes/obstacle_cone_ab.json`。

**指令**：
```bash
export CUDA_VISIBLE_DEVICES=1   # 或視 nvidia-smi 挑目前較閒的那張
/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python research/scripts/select_demo_poses.py \
  --x -0.05 0.05 0.15 0.25 0.35 --y 0.05 0.15 0.25 0.35 --z 0.35 0.42 0.50 0.58 \
  --num-seeds 64 --position-tolerance 0.01 --orientation-tolerance 0.08 \
  --min-joint5-margin-rad 0.15 --include-pole \
  --out research/configs/demo/ab_poses.yaml --report out/demo/select_demo_poses_scan.json
```

### 8.2 新增的共用模組——已實作＋單元測試全過

```
research/src/mpg/urdf_fk.py
  UrdfChainFk           # 純 numpy URDF FK，跟 tools/verify_fk_consistency.py（已重構成呼叫這個模組，
                         #   仍然 PASS，數字不變）、cuRobo FK、yourdfpy 三方交叉驗證過
  classify_approach_direction / quaternion_angular_distance_xyzw  # 「approach ≈ +Z(上)，偏差2.1°」這種
                         #   報告用的工具函式，reactive.py 和 tools/send_joint_cmd.py（下一步）共用

research/src/mpg/curobo_bridge/frames.py
  approach_direction_to_quat_xyzw()   # 新增：命名方向（"up"/"left"/...）-> pinch_center 目標姿態
  NAMED_DIRECTION_ALIASES             # 方向名稱 -> 世界座標向量，select_demo_poses.py 和
                                       #   waypoints.py 共用同一份，不會兩邊定義出不同的「上」

research/src/mpg/curobo_bridge/waypoints.py     # 新增，純 Python，不 import curobo
  Waypoint / WaypointSequencer   # 狀態機：approaching -> holding -> 下一個，或逾時強制前進
  load_waypoints_from_yaml() / load_waypoints_from_ab_poses()  # 後者直接讀 8.1 的 ab_poses.yaml，
                                       #   讓 reactive.py 的避障情境和之後的 6 軸輸入工具共用同一份「A/B 是什麼」

research/src/mpg/ros_pointcloud.py   # 新增，PointCloud2 手動組裝（x,y,z float32 + r,g,b uint8）
                                       #   從 reactive.py 抽出來，因為 reactive.py 在模組層級 import curobo，
                                       #   沒辦法在只有 rclpy、沒有 curobo 的 venv 裡單獨測
```

單元測試：`research/tests/test_{urdf_fk,waypoint_sequencer,ros_pointcloud}.py` + `test_curobo_frames.py`/`test_reactive_controller.py` 新增的 case。2026-09-24 實測：

```bash
cd research && PYTHONPATH=src /mnt/HDD4/wyattsheu/env_robot129_research/bin/python -m unittest discover -s tests
# -> Ran 220 tests ... OK (skipped=7，全部是「這個 venv 沒裝 yourdfpy/curobo/rclpy」的預期 skip)

PYTHONPATH=src /mnt/HDD4/wyattsheu/env_robot129_ros/bin/python -m unittest tests.test_ros_pointcloud -v
# -> 2 tests OK，用 sensor_msgs_py.point_cloud2.read_points（獨立於我們自己的 struct.pack_into）解碼覆核過欄位對不對

CUDA_VISIBLE_DEVICES=1 PYTHONPATH=src /mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python -m unittest tests.test_curobo_frames tests.test_urdf_fk -v
# -> 23 tests OK，含 yourdfpy 交叉驗證，零 skip
```

**過程中抓到的真實 cuRobo 陷阱，記在 `reactive.py` 模組 docstring**：`VoxelGrid.get_occupied_voxels()`（`Mapper.compute_esdf()` 回傳物件上的方法）不能直接呼叫——它需要 `.xyzr_tensor`，但 `compute_esdf()` 回傳時這欄位是 `None`；就算自己補呼叫 `create_xyzr_tensor()`，`feature_tensor` 是沒攤平的 `(nx,ny,nz)` 3D grid，跟 `get_occupied_voxels()` 內部假設的攤平形狀對不上，會噴 shape mismatch（親自撞過，不是看文件猜的）。而且沒觀測到東西的體素預設值是 `10000.0`（標準 cuRobo ESDF 慣例：負值=障礙物內部，正值=自由空間，對照 `curobo/examples/getting_started/volumetric_mapping.py` 自己的視覺化「blue=inside, white=zero, red=outside」核對過），如果真的接上 `get_occupied_voxels()` 的預設 threshold，會把整個沒觀測到的區域誤判成「佔據」。正確作法（跟著 cuRobo 官方 `volumetric_mapping.py` 範例自己畫點雲用的方法一樣）是呼叫 block-sparse TSDF integrator 自己的 `mapper.integrator.extract_occupied_voxels(surface_only=...)`，包成 `LiveEsdfReactiveController.occupied_voxels()`。用真實 fixture（`research/tests/fixtures/scene_camera_home/`）測過不會 crash、回傳形狀正確；這個 fixture 本身在這個 grid 範圍內沒有真的觀測到障礙物表面（跟 §7.4 記錄的「HOME 姿態下水平相機看不到近處物體」一致），所以只驗證了「不會 crash、格式對」，**沒有驗證非零數量的真實案例**——留給 §8.6 表格裡「`reactive.py` 跟真實 Isaac 的端對端」那一項，用真的有障礙物的情境（dynamic_stick / static_cylinder，§8.4 已經驗證過這兩個情境本身能正確生成障礙物）做。

### 8.3 `reactive.py` 擴充——已實作，CLI 用 `--waypoints research/configs/demo/ab_poses.yaml` 對真實 robot129.yml 跑過（沒有 live Isaac，驗證到「正確載入＋正確逾時失敗」為止）

新增 `--waypoints <path>`（自動判斷是專用 waypoints.yaml 還是 8.1 的 ab_poses.yaml schema）、`--loop`、`--position-tolerance-m`、`--orientation-tolerance-rad`、`--report-out`。沒給 `--waypoints` 時完全維持原本 `--goal-a/--goal-b` 計時切換的行為，不影響任何既有呼叫。

新增發布（`/robot129_sim/reactive/{goal,status,obstacle_voxels}`），對應計畫的視窗 1 需求。

**已驗證**（`PYTHONPATH=research/src:/mnt/HDD4/wyattsheu/env_robot129_ros/lib/python3.12/site-packages`，讓 curobo venv 的 python 也拿得到 rclpy）：
```bash
export CUDA_VISIBLE_DEVICES=1
/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python research/src/mpg/curobo_bridge/reactive.py \
  --waypoints research/configs/demo/ab_poses.yaml --duration-s 5
# -> [curobo_reactive] loaded 2 waypoint(s) from research/configs/demo/ab_poses.yaml: ['A', 'B']
#    [curobo_reactive] waiting for first depth + info + tf + joint_states ...
#    [curobo_reactive] FAIL: did not receive all required topics within 30s
```
這是預期行為（沒開 Isaac，本來就等不到 topic）——證明了 waypoints 載入、controller 建置、rclpy node/publisher 建置全部沒問題。**跟真實 Isaac 一起跑、確認真的會避開障礙物、抵達 A/B、姿態朝上/朝左，還沒做**，是下一步。

### 8.4 `sim/scripts/run_robot129_ros_webrtc.py` 擴充——已實作＋對真實 Isaac 驗證，含截圖

**新增**：
- Overview 相機（原本只餵 `--record` 的影片，現在也發布到 ROS）：`/robot129_sim/overview_camera/{color/image_raw,camera_info}`，RGB-only，`frame % 8` 節流（約 wrist/scene 相機的一半更新率）。視角可調（`research/configs/scene_camera.yaml` 新增 `overview_camera` 段落），預設值跟改之前的硬編碼值逐位元組相同。
- `--scene-manifest` 新增三種形狀：`primitive_cylinder`／`primitive_cone`／`primitive_sphere`（`radius_m`、`height_m`），走跟 `primitive_box` 一樣的路徑（capture-back 讀即時位置）。
- `--scene-manifest` 開放給 `--scene dynamic_stick`（原本只有 `pick_place*` 能用）。
- `--stick {swing,none}`（+`--stick-period-s`/`--stick-amplitude-m`）：`none` 完全不生成棍棒。
- **修了一個順手發現的真實 gap**：manifest 障礙物原本沒有接觸感測器，`static_cylinder` 情境沒辦法量到「有沒有碰撞」。改成 `obstacle_prim_paths`（棍棒 + 全部 manifest 物件）統一過濾，`publish_stick_and_contacts()` 的 TF 發布（只有棍棒存在時）跟力量發布（只要有任何障礙物感測器就發布）拆開，manifest 的所有形狀都加上 `activate_contact_sensors=True`。
- `--target-object <id>`：讀 `research/configs/grasp_targets.json`（`cube_35`(預設)/`tall_block`/`can`/`flat_box`/`ball`），只對純 `pick_place` 生效（`pick_place_hammer`、`pick_place_counter` 有自己的目標幾何，忽略這個旗標）。`reset_scene` 會用同一個 `target_drop_z`（依形狀算出來的正確落下高度，不是固定用方塊的），`scene_state` 會回報 `target_object` 欄位。

**對真實 Isaac 驗證過**（headless `--record-only`，`tools/start_robot129_grasp_sim.sh` 的 GPU 自動選擇邏輯在這台機器目前的負載下會選到幾乎沒剩多少可用顯存的那張卡導致 OOM，改成手動指定 `CUDA_VISIBLE_DEVICES=0`——這個特定的選卡瑕疵不在這次範圍內，沒有去動那支工具腳本）：

1. `--scene pick_place --target-object can --scene-camera pole`：READY、`scene_state` 正確回報 `"target_object": "can"`、`objects/target_cube/pose` 的 z=0.045（剛好是圓柱高度 0.09 的一半，形狀/落地高度算對了）、overview/wrist 相機截圖親眼確認黃色圓柱正確生成。
2. `--scene dynamic_stick --stick none --scene-manifest research/configs/scenes/obstacle_static_cylinder.json --scene-camera pole`：READY、overview 截圖確認橘色圓柱障礙物生成、沒有棍棒、`/robot129_sim/contacts/gripper_base` 正確發布 0N（接觸感測器接上了新障礙物）、`capture_scene_manifest` 服務把圓柱的 xy/z/radius/height 完整讀回、跟原始 manifest 完全吻合。
3. `--scene dynamic_stick --stick none --scene-manifest research/configs/scenes/obstacle_cone_ab.json --scene-camera pole`：READY、overview 截圖確認粉紅色錐形障礙物正確生成。
4. `--scene pick_place`（不帶任何新旗標）：READY、`objects/target_cube/pose` 的 z=0.0175（跟改之前完全一致）、`scene_state` 回報 `"target_object": "cube_35"`、overview 截圖確認紅色方塊跟改之前一模一樣——**確認沒有 regression**。

四次都用 `research/tests/`-style 的一次性 rclpy 訂閱腳本抓 wrist/scene/overview 三張截圖，親眼看過，不是只驗證「topic 有資料」。

**還沒做**：跟真實 Isaac 一起跑 `reactive.py --waypoints`（見 8.3）；`--target-object` 的 `tall_block`/`flat_box`/`ball` 只驗證了 JSON schema 語法，沒有真的在 Isaac 裡生成過；`grasp_targets.json` 的 `mass_kg` 是用既有方塊的密度換算出來的估計值，不是量測值。

### 8.5 `tools/send_joint_cmd.py`——已實作＋對真實 Isaac 驗證，含 A/B 完整跑過

6 軸關節輸入工具（`--joints j1..j6 [--deg]`／`--sequence <yaml>`／`-i` 互動模式），跟 `tools/send_robot129_ros_pose.py` 同一套 JointTrajectory + 穩定偵測手法（連續 5 個樣本誤差 < tolerance 才算到位），但接受任意 6 個數字，不是只有 5 個寫死的預設姿勢。每一步都印出「每軸命令值/實際值/誤差」表，並用 `mpg.urdf_fk` 算出 pinch_center 位置跟 approach 軸方向（例如「approach ~= +Z (up), off by 3.0 deg」）。啟動時檢查 `/robot129_sim/piper/joint_cmd` 有沒有 publisher（`reactive.py` 在跑），有的話直接拒絕執行，不會送出一個馬上被蓋掉的命令。

**一個真實發現，修正了原本的假設**：一開始用 `SETTLE_TOLERANCE_RAD = 0.01`（rad），示範點 A（`joint1=-2.158 rad`，離 HOME 很遠）就算等到 60 秒 wall-clock 還是卡在 `max_error=0.013`，穩定不下去——不是逾時問題，是這顆 PD controller（`stiffness=800, damping=80`，沒有重力補償）對這種姿態本來就有這個量級的穩態誤差。查了一下，`ros2_ws/src/robot129_sim_execution/config/adapter_params.yaml` 的 `arm_goal_tolerance_rad` 早就因為同一類問題設成 `0.03`——改用這個既有的、已經驗證過的容忍值，不是自己發明一個更嚴但這顆 controller 對某些姿態根本達不到的數字。

**對真實 Isaac 驗證過**（`--scene pick_place`，headless）：
```bash
bash tools/start_robot129_grasp_sim.sh --scene pick_place
# 另開一個 shell，source 好 ROS 環境後：
python3 tools/send_joint_cmd.py --joints 0.5 1.4 -1.5 0.2 0.3 0.5 --duration 2.5
# -> 6 軸全部 PASS，誤差 <0.007 rad
python3 tools/send_joint_cmd.py --sequence <A/B-only sequence>
# -> A_gripper_up: approach ~= +Z (up) (off by 3.0 deg), pinch_position=(0.253, 0.360, 0.578)，目標 (0.25,0.35,0.58)
#    B_gripper_left: approach ~= +Y (left) (off by 4.5 deg), pinch_position=(0.048, 0.352, 0.347)，目標 (0.05,0.35,0.35)
```
**這直接證實了使用者最初的需求**：「A 點朝上、B 點朝左，明確確認手臂有這個功能」——不是理論上的 IK 解，是真的送進 Isaac、量到手臂穩定到位、用獨立的 FK 算出實際夾爪方向，誤差 3–4.5 度。

`research/configs/demo/joint_sequence.yaml`（已提交）：HOME → 六軸各自單獨掃一次（每次只動一軸，方便看單一軸的效果）→ 回 HOME → A → B，共 9 步；只實際跑過最後兩步（A/B），前面逐軸掃描的部分還沒有真的在 Isaac 上跑過一次。

### 8.7 `tools/rerun_dashboard.py`——四視窗介面本體，已實作＋對真實 Isaac 驗證（含 WebRTC 同時開）

純 ROS 訂閱者（不發布、不控制手臂，跟 `research/scripts/capture_scene.py` 同一個「dashboard 不碰控制」的慣例），用 Rerun 0.23 的 `serve_web()`（web viewer port 9090、gRPC port 9876，兩個都預設只綁 localhost）同時：

- **視窗 1**（`world/points` + `world/obstacle_voxels`）：腕上相機＋後方相機的深度反投影融合彩色點雲，加上 `reactive.py`（若有在跑）即時發布的 ESDF 障礙體素；另外用 `mpg.urdf_fk` 從 `/robot129_sim/joint_states` 即時算出 `world/robot/pinch_center` 的座標軸，方便對照「偵測到的東西」跟「手臂實際在哪」。
- **視窗 2/3/4**：`cameras/overview` / `cameras/rear` / `cameras/wrist`，JPEG 壓縮（quality 85）後才記錄，見下面的真實踩坑。
- 下方分頁：`joints`（6 軸命令值 vs 實際值時序）、`contacts`（各 link 接觸力大小，訂閱 `link3`~`link8`+`gripper_base` 全部可能的名稱，沒發布的自然不會有資料）、`status`（`scene_state` + `reactive/status` 的 JSON 原文）。
- 同時輸出兩個地方：即時 gRPC/web（給你當下看）+ 存檔 `.rrd`（預設 `out/demo/dashboard/<timestamp>/dashboard.rrd`，`--no-save` 可關掉）——用兩個獨立的 `rr.RecordingStream`（`.serve_web()` 一個、`.save()` 一個），每筆資料呼叫兩次 log，這是 Rerun 0.23 公開 API 目前唯一支援「同時直播+存檔」的方式（沒有 `set_sinks` 這種一次設多個 sink 的函式）。

**跑在哪個 venv**：`env_robot129_realstack`——這是目前唯一同時裝了 `rerun_sdk` 跟 `rclpy`（疊在 `env_robot129_ros` 上）的環境。⚠️ **真的踩過的坑**：如果你在這個 venv 裡用 `micromamba run -p env_robot129_realstack bash -c 'source env_robot129_ros/setup.bash; python3 ...'`，`source setup.bash` 會把 `env_robot129_ros/bin` 加到 `PATH` 前面，蓋掉 `python3` 應該指向的 `env_robot129_realstack/bin/python3`，變成用沒裝 rerun 的那個 python 執行、直接 `ModuleNotFoundError: No module named 'rerun'`。**一定要用完整路徑 `/mnt/HDD4/wyattsheu/env_robot129_realstack/bin/python3`，不要依賴 `source` 完之後的 `python3`**——`tools/start_demo_dashboard.sh`（見下）已經照這個方式寫好，直接用它就不會踩到。

**過程中修的另一個真實問題**：一開始三個相機視窗沒壓縮，直接 `rr.Image(array)` 記錄原始像素，30 秒錄到快 29MB，換算下去長一點的 demo 錄影會很誇張。改成 `.compress(jpeg_quality=85)`，同樣時長掉到約十分之一大小，畫質肉眼看不太出差別。

**對真實 Isaac 驗證過**（2026-09-24，headless `pick_place --scene-camera pole`）：
```bash
bash tools/start_robot129_grasp_sim.sh --scene pick_place --scene-camera pole
bash tools/start_demo_dashboard.sh --duration-s 30
curl -sS -o /dev/null -w "HTTP %{http_code}\n" http://localhost:9090/   # -> HTTP 200
```
存檔的 `.rrd` 用 `rr.dataframe.load_recording()` 讀回來檢查過 schema，56 個 component column，涵蓋 `/cameras/{overview,rear,wrist}`、`/world/points`、`/world/robot/pinch_center`、`/joints/{actual,commanded}/joint1-6`、`/contacts/{link7,link8}`、`/status/{scene,gripper}`——不是只驗證「檔案有變大」，是真的把記錄的內容目錄核對過。`reactive.py` 沒開的這次測試裡，`world/obstacle_voxels`／`status/reactive` 自然沒有資料（那兩個 topic 不存在），符合預期，不是漏寫。

**2026-09-24 補的真實 bug——瀏覽器開網址只看到 Rerun 官方歡迎畫面，不是這個 dashboard**：原本的程式碼用 `RecordingStream.serve_web()` 這個一次做完 gRPC+web viewer 的便利函式，`open_browser=False`（這支腳本本來就是無頭環境跑的），然後只印出 `http://localhost:9090` 叫人自己開。結果瀏覽器開起來是 Rerun 官方的「Visualize multimodal data / Log data with the Rerun SDK...」介紹頁，不是四視窗畫面。查了 viewer 自己送出來的 HTML/JS（不是猜的）：它用 `new URLSearchParams(window.location.search).getAll("url")` 讀網址上的 `?url=` 參數來決定要連去哪個 gRPC 位址，沒有這個參數就停在「沒有連線」的歡迎畫面。修法：改成分開呼叫 `serve_grpc()`（拿到回傳的連線 URI，格式是 `rerun+http://127.0.0.1:<grpc_port>/proxy`）+ `serve_web_viewer(connect_to=<那個 URI>)`，自己組出正確的完整網址（`http://localhost:9090/?url=<URI 做過 URL-encode>`）印出來；`start_demo_dashboard.sh` 也跟著改成從 log 撈這行正確網址印給你，不再自己組一個錯的。修好後實測：`curl` 打這個完整網址回 200，兩個 port（9090/9876）都在聽，viewer 的 JS 能正確解析參數（讀原始碼確認，沒有另外用瀏覽器目視驗證畫面內容）。

### 8.8 WebRTC 維護＋跟 dashboard 同時跑——已實作＋對真實 Isaac 驗證

**修的問題**：`tools/start_robot129_ros_webrtc.sh` 原本只轉傳 `--scene`/`--scene-manifest`/`--wrist-camera-model` 三個參數，8.1–8.5 新增的 `--scene-camera`/`--target-object`/`--stick`/`--stick-period-s`/`--stick-amplitude-m` 全部不會被轉傳過去——跟 2026-09-20 那次 `--scene` 沒轉傳的 bug 是同一個模式（這個腳本自己的參數白名單，沒列到的新參數會被靜默丟掉），已經補上。

**過程中抓到的第二個真實 bug，也修了**：`capture_viewport_probe()`（WebRTC 啟動時的視窗驗收檢查）原本寫死檢查「畫面裡有沒有紅色像素」，假設場景一定有紅色的抓取目標。這個假設被 8.1–8.4 的新功能打破了兩種情況：`--scene dynamic_stick`（棍棒是橘色、或 manifest 障礙物顏色不定）、`--target-object` 選到非預設顏色（例如 `can` 是黃色）。實測 `--scene dynamic_stick --stick none --scene-manifest .../obstacle_static_cylinder.json --scene-camera pole` 過 WebRTC 啟動：`VIEWPORT_PROBE FAIL - red_target_fraction: 0.0`——但截圖打開一看，圓柱障礙物根本清清楚楚在畫面正中央，是驗收邏輯本身的假設錯了，不是渲染壞了。

第一次修法（改成算「畫面裡跟目標顏色的歐氏距離夠近」的像素比例）也不夠：同一張截圖裡，橘色圓柱 (0.85,0.35,0.10) 材質、實際渲染出來的像素平均是 (230,201,133)（偏淡黃、RTX 燈光把材質打亮/去飽和很多），跟原始材質色的歐氏距離高達 ~155，隨便設的門檻 60 直接被打臉。**最終修法**：改成比色相（hue）而不是比 RGB 距離——用上面同一組真實資料驗證過，色相只飄移約 22°（遠比亮度/飽和度穩定），改用「色相差 < 35 度 且 該像素夠飽和（chroma>20，排除灰階雜訊）」，同一批真實截圖（圓柱 2.5% 像素匹配、原本的紅色方塊 0.25% 像素匹配）都遠遠超過門檻，`marker`／`pick_place`／`pick_place --target-object can`／`dynamic_stick --stick none` 四種情境過 WebRTC 全部驗證過 `VIEWPORT_PROBE PASS`，預設情境（沒帶任何新參數）像素比對結果跟改之前幾乎一樣（0.0022 vs 原本應該相近的量級），確認沒有 regression。

**WebRTC 可以跟 dashboard 同時開，已經真的一起跑過**：兩個是完全獨立的行程，WebRTC 只佔 port 49100（Isaac 自己的互動直播），dashboard 只佔 9090/9876（讀 ROS topic，不碰 GPU 渲染），互不衝突。

#### 執行指令（把這節當成之後每次要開demo 的操作手冊）

**只看 WebRTC，不用 dashboard**：
```bash
bash tools/stop_any_webrtc.sh   # 確保 49100 是空的
bash tools/start_robot129_ros_webrtc.sh --scene pick_place --scene-camera pole
# 瀏覽器開 https://140.113.203.85:49100（或問你要的 client 連法，跟這個 repo 既有 WebRTC 流程一樣）
bash tools/stop_robot129_ros_webrtc.sh    # 結束
```

**WebRTC + 四視窗 dashboard 同時開（你要的用法）**：
```bash
bash tools/stop_any_webrtc.sh
bash tools/start_robot129_ros_webrtc.sh --scene pick_place --scene-camera pole   # 視情境換場景/參數，見下表
bash tools/start_demo_dashboard.sh          # 預設跑到你按 Ctrl-C／手動 stop，不用 --duration-s
# 本機瀏覽器：WebRTC 開 140.113.203.85:49100；dashboard 開「上面這行指令印出來的網址」（見下面的重要提醒）
# 遠端瀏覽器：dashboard 這邊要先 ssh -L 9090:localhost:9090 -L 9876:localhost:9876 <這台機器>，
#            WebRTC 本來就是設計成可以直接連公網 IP，不用 tunnel

# 結束：
bash tools/stop_demo_dashboard.sh
bash tools/stop_robot129_ros_webrtc.sh
```

⚠️ **重要，2026-09-24 真的有人踩到**：dashboard 網址**不是**單純的 `http://localhost:9090`——直接開那個網址只會看到 Rerun 官方的空白歡迎畫面（"Visualize multimodal data / Log data with the Rerun SDK..." 那種介紹頁），不是這個 dashboard 的四視窗畫面。原因：Rerun 的網頁 viewer 需要網址帶一個 `?url=<gRPC 連線位址>` 的查詢參數才會自動連上資料流（讀了 viewer 自己的 HTML/JS 才確認的：`new URLSearchParams(window.location.search).getAll("url")`，沒有這個參數就停在「沒有連線」的歡迎畫面）。**`bash tools/start_demo_dashboard.sh` 執行完，直接複製它印出來的那一行完整網址**（長得像 `http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy`），不要自己手動輸入 `localhost:9090`。用 SSH tunnel 連遠端看的時候也一樣——tunnel 完，瀏覽器開的還是這個完整網址（把 `localhost` 留著，不要換成這台機器的 IP，因為 tunnel 是轉發到你自己電腦的 localhost）。

**只跑 dashboard，不用 WebRTC（GPU 更省，沒有互動視窗）**：
```bash
bash tools/start_robot129_grasp_sim.sh --scene pick_place --scene-camera pole   # headless，--record-only
bash tools/start_demo_dashboard.sh
# ...
bash tools/stop_demo_dashboard.sh
bash tools/stop_robot129_grasp_sim.sh
```

**五個情境對應的 `--scene`/manifest 參數**（`start_robot129_ros_webrtc.sh` 跟 `start_robot129_grasp_sim.sh` 都吃這些參數，兩邊都轉傳過）：

| 情境 | 指令參數 |
|---|---|
| 揮動棍棒 | `--scene dynamic_stick --scene-camera pole` |
| 靜態圓柱 | `--scene dynamic_stick --stick none --scene-manifest "$(pwd)/research/configs/scenes/obstacle_static_cylinder.json" --scene-camera pole` |
| A→B 繞過錐形障礙物 | `--scene dynamic_stick --stick none --scene-manifest "$(pwd)/research/configs/scenes/obstacle_cone_ab.json" --scene-camera pole`（另開一個 shell 跑 `reactive.py --waypoints research/configs/demo/ab_poses.yaml`，見 §8.3） |
| 多種目標抓取 | `--scene pick_place --target-object {cube_35,tall_block,can,flat_box,ball}` |
| 6 軸關節輸入 | 任一場景 + 另開 shell 跑 `tools/send_joint_cmd.py`（見 §8.5） |

**GPU 選卡注意**：`start_robot129_grasp_sim.sh`／`start_robot129_ros_webrtc.sh` 自動選卡的邏輯（依 `utilization*100+memory/100` 排序）在這台機器目前的負載型態下可能選到「利用率低但顯存快滿」的那張卡而 OOM（2026-09-24 實測遇過），這個腳本本身的瑕疵不在這次修的範圍內。踩到的話手動 `export CUDA_VISIBLE_DEVICES=<較多剩餘顯存的那張>` 再執行腳本內容（或直接看 `nvidia-smi` 挑卡，跳過腳本的自動選卡邏輯）。

**⚠️ 真的發生過的意外——`stop_robot129_grasp_sim.sh` 誤殺別人正在跑的 WebRTC session（已修）**：這支腳本自己的 `pkill -f` fallback 用的 pattern 原本只有 `sim/scripts/run_robot129_ros_webrtc.py --bundle $root`——這剛好是 headless（`--record-only`）和 WebRTC（`--livestream`）兩種啟動方式共用的字串（同一支 python 檔案，只差旗標），所以呼叫這支「應該只管 headless」的停止腳本，實際上把使用者自己另外開的、完全無關的 WebRTC session 也一起殺掉了。跟 `stop_any_webrtc.sh` 檔頭已經記錄的「2026-09-20 學過一次，不能讓停 WebRTC 的腳本連帶殺掉 headless sim」是同一類問題，只是方向相反這次才踩到。**已修**：pattern 加上 `--device cuda:0 --record-only`（只有 headless 啟動指令才有這段），確認過現在對正在跑的 WebRTC session 執行 `pgrep -f` 用新 pattern 不會誤判。教訓：這個 repo 裡任何用 `pkill -f`／`pgrep -f` 做「認出我自己啟動的那個 process」的地方，都要檢查 pattern 是不是「共用底層腳本、只差旗標」的兩支姊妹腳本會不會互相誤傷——`start_robot129_grasp_sim.sh` 和 `start_robot129_ros_webrtc.sh` 就是這種關係，以後新增類似的一對 start/stop 腳本要小心同一個坑。

### 8.9 Phase 5：`tools/run_demo_scenario.sh`（一鍵情境腳本）——已實作＋五個情境跑過三個，過程中修了三個真實 bug

`tools/run_demo_scenario.sh <stick|static_cylinder|p2p_cone|grasp|joints> [--webrtc] [--target ID] [--duration-s N] [--out-dir DIR] [--keep-running]`：停掉任何殘留的 session → 啟動 Isaac（headless 或 `--webrtc`）→ 啟動 dashboard（失敗不中斷，只是警告——dashboard 是輔助視覺化，不是情境成敗的一部分）→ 呼叫 `/robot129_sim/recording` 開始錄影 → 跑該情境的驅動程式 → 停止錄影、等 ffmpeg、把 `video.mp4`/`wrist_video.mp4`/`scene_video.mp4`/`dashboard.rrd`/驅動程式的 log 和 report 收進 `out/demo/<情境>/<timestamp>/` → 關掉 dashboard 和 Isaac。**這支腳本不會自動判斷計畫裡列的每一條通過標準**（例如「接觸力全程 0N」），那些數字在收集回來的 log/report 裡，還是要人看一眼或另外寫分析。

**配套完成的**：`tools/generate_s2_cube_candidates.py` 泛化成讀 `research/configs/grasp_targets.json`（`--target <id>`），`tools/run_curobo_grasp_demo.sh` 加上 `--target` 轉傳。

#### 泛化 `generate_s2_cube_candidates.py` 時抓到的真實坑

第一版泛化把 `cube_surface_points()` 直接改寫成一個「通用 box 函式」，用參數取代原本寫死的 `CUBE_X/CUBE_Y/CUBE_Z/CUBE_SIZE`——結果對照 `cube_35`（沒帶 `--target`）的輸出跟改之前的 git HEAD 版本做 diff，**點的數量一樣，但每個候選的 `edge_point_indices` 完全不同、分數也跟著變了**。原因：原本的寫法是單一巢狀迴圈 `for a in lin: for b in lin:`，每次迭代把 6 個面（4 側面+上下）的點交錯著 append；我改寫成「先掃兩個側面、再掃另外兩個側面、最後掃上下面」的分段寫法，點集合在幾何上等價，但疊代順序不同，導致 `generate_grasp_candidates()`（依點的 index 做邊緣/對蹠檢測）產出不同結果。**修法**：`cube_35` 這條路徑保留原始函式一字不改（只是把模組層級變數換成參數），新的形狀（`tall_block`/`flat_box`/`can`）另外用一個新函式處理，不跟舊函式共用同一份邏輯。這是這次全程對照「改之前的輸出」做 regression diff 才抓到的，光看程式碼、跑得動、候選看起來合理，是看不出來的。

**驗證過的候選生成結果**（`/mnt/HDD4/wyattsheu/env_robot129_research/bin/python tools/generate_s2_cube_candidates.py <out> --target <id>`）：

| target | 形狀 | 結果 |
|---|---|---|
| `cube_35`（預設） | box | 8/8 accepted，跟改之前的輸出逐位元組相同 |
| `tall_block` | box | 8/8 accepted |
| `can` | cylinder | 8/8 accepted |
| `flat_box` | box | **0/8 accepted，全部 `TABLE_CLEARANCE`** |
| `ball` | sphere | 明確拒絕執行（`sphere` 不在支援清單，印出原因，不是靜默產生垃圾候選） |

`flat_box`（6×3×2.5cm）用現有的 `generate_grasp_candidates()` 參數組合完全生不出候選——這是真實限制，不是這次沒調好，沒有為了讓它「看起來能用」去動 `grasp_candidates.py` 的參數（那是共用邏輯，可能影響其他已驗證過的候選生成）。`grasp` 情境目前只能穩定用 `cube_35`/`tall_block`/`can`。

#### `run_demo_scenario.sh` 開發過程中抓到的兩個真實 bug（都修了）

1. **`reactive.py` 找不到 `rclpy`**：`reactive.py` 同時需要 `curobo`（只在 `env_robot129_curobo`）跟 `rclpy`（只在 `env_robot129_ros`），兩個 venv 都要。第一版直接呼叫 `$curobo_python reactive.py`，沒有把 `env_robot129_ros` 的 site-packages 疊上去，實測直接 `ModuleNotFoundError: No module named 'rclpy'`——跟 dev guide 前面 §7.8 已經記過的組合方式（`PYTHONPATH=research/src:.../env_robot129_ros/.../site-packages`）搞混了，這次補上。

2. **`start_demo_dashboard.sh` 的就緒偵測不可靠，非常值得記錄**：一開始用 `grep 'serving web viewer' server.log` 判斷 dashboard 是否就緒，實測發現這個訊號完全不可靠——**同一支腳本、不同次執行，`server.log` 有時候在 process 明顯已經在服務（`ss` 看得到 port 9090 在聽）的情況下仍然維持空白超過 30 秒**，原因目前沒有完全查清楚（懷疑是 rerun 的 `serve_web()` 內部起了自己的 server thread/native 程式碼，跟這支 script 的 buffered stdout 寫入互相干擾，但沒有深入到 rerun 原始碼層級確認）。另外，用來判斷「process 還活著」的 `$!`（`nohup setsid ... &` 之後拿到的 PID）也不可靠——實測 `kill -0 "$!"` 在兩個方向都錯過：一次是 process 明明還在跑、`kill -0` 卻回報「不存在」；另一次是回報「還活著」長達整個 30 秒逾時視窗、但 dashboard 其實從來沒起來。這跟 `stop_robot129_grasp_sim.sh` 檔頭已經記錄的「Isaac/Kit 子行程有時候跟 launcher 的 setsid process group 對不上」是同一類問題（`setsid` 需要 fork 時，捕捉到的 `$!` 是短命的 wrapper，不是真正長跑的孫行程）。**最終修法**：就緒判斷改成直接 `ss -lnt | grep ':9090 '` 檢查 port 有沒有在聽（比等一行特定的 log 字串更直接、也更可靠），存活判斷改成 `pgrep -f "tools/rerun_dashboard.py"`（比對完整指令列，不依賴任何特定 PID）。修好後實測：0.6 秒內回報 READY，而不是原本的 timeout。

**還沒修的已知小缺口**：`stop_demo_dashboard.sh` 送 `SIGTERM` 關閉 dashboard 時，rerun 的 `.save()` sink 不一定會把 `.rrd` 完整落盤——實測 `stick`/`p2p_cone` 兩次情境跑完後，收集到的目錄裡都沒有 `dashboard.rrd`（即時 web 畫面在跑的當下是正常的，只有「存檔」這個附加功能在關閉時遺失資料）。優先度較低（影片錄影正常，這只影響離線用 `.rrd` 回放），先誠實記下來，之後有空再查。

#### 三個情境的真實執行結果（2026-09-24，GPU 當時完全閒置）

**`joints`**（`bash tools/run_demo_scenario.sh joints`，跑 `research/configs/demo/joint_sequence.yaml` 完整 9 步）：8/9 步 PASS，包含最後的 A/B 兩步——`A_gripper_up`: approach ≈ +Z(上)，偏差 3.0°；`B_gripper_left`: approach ≈ +Y(左)，偏差 4.4°。唯一失敗的是 `joint2_sweep`（把 joint2 從 HOME 的 1.20 rad 轉到 2.20 rad）：`max_error=0.063 rad`，超過 `0.03 rad` 容忍值——這是這顆 PD controller（無重力補償）對某些姿態的真實穩態誤差，不是逾時或腳本問題，值得之後調查是不是這個特定姿態剛好卡在某個 configuration 附近誤差特別大。整趟收集到 `video.mp4`、`wrist_video.mp4`、`joint_response.json`。

**`stick`**（`bash tools/run_demo_scenario.sh stick --duration-s 30`）：完整跑完，`reactive.py` 統計 `steps=137, none_steps=0, esdf_updates=35`——137 次 MPC 求解全部成功（0 次「沒有可用動作」），沒有另外對接觸力做逐步記錄（見上面「這支腳本不會自動判斷通過標準」的說明），但跟 §7.8 已經驗證過的「同一套 controller 在 dynamic_stick 情境下 0 碰撞」結果一致。收集到三路影片（overview/wrist/scene，因為這個情境有 `--scene-camera pole`）。

**`p2p_cone`**（`bash tools/run_demo_scenario.sh p2p_cone`，預設 waypoint 逾時 20 秒）：**FAIL**——A、B 兩個 waypoint 都在 20 秒 wall-clock 逾時視窗內被強制跳過，最終位置誤差分別是 0.41m／0.51m，角度誤差 1.62／1.96 rad，離目標還很遠。`stats={'steps': 191, 'none_steps': 0, 'esdf_updates': 49}`——MPC 本身求解全部成功（沒有 solver 失敗）。**一開始懷疑是 20 秒 wall-clock 對這台機器的即時速度不夠，但拉長到 120 秒重跑後證明不是這個原因**（見下面的延伸測試）——真正的結論放在下面那段。

**額外驗證（追蹤 p2p_cone 的逾時問題）——結論：不是逾時不夠長，是 MPC 真的沒有在收斂**：獨立跑 `reactive.py --waypoints ab_poses.yaml --waypoint-timeout-s 120 --waypoint-hold-s 2.0`（不透過 `run_demo_scenario.sh`，同一顆閒置 GPU，把每個 waypoint 的逾時從預設 20 秒拉長到 120 秒，6 倍）。結果：**A、B 兩點還是都逾時**，而且最終誤差比 20 秒逾時那次還大（A: 位置誤差 0.62m／角度誤差 2.64 rad，B: 位置誤差 0.61m／角度誤差 1.87 rad，20 秒那次分別是 0.41/1.62 和 0.51/1.96）。整趟跑了 1188 次 MPC 控制步，`none_steps=0`（每一步 solver 都有解出動作，不是 solver 崩潰），`esdf_updates=298`（ESDF 有持續更新，不是感知端卡住）。

這排除了「wall-clock 逾時不夠長」這個假設——如果只是單純的移動距離長、需要更多步驟，1188 步、超過 4 分鐘的持續求解應該早就看得到誤差穩定下降的趨勢，但實際上誤差在拉長 6 倍逾時後反而略為變大，代表 MPC 沒有朝目標穩定收斂，比較像是卡在某個 local minimum 附近來回、或是被錐形障礙物的碰撞代價卡住走不過去。這是 `reactive.py` waypoints 模式本身（或這組 A/B 目標姿態、或錐形障礙物的擺放）需要回頭調查的真實問題，**不是這次 Phase 5 orchestration 腳本的 bug**，超出這輪修 `run_demo_scenario.sh`/`generate_s2_cube_candidates.py` 的範圍，留給下一輪處理。已知的初步懷疑方向（都還沒驗證）：① A 點本身雖然 joint5 餘裕足夠（§8.1 記錄 0.63 rad），但 IK 解出來的關節組合到起點的路徑可能剛好需要繞過錐形障礙物、又要維持特定夾爪朝向，兩個約束疊在一起讓可行解空間變得很窄；② `optimizer_collision_activation_distance`／MPC 的收斂速度參數，這組實驗目前用的是 `reactive.py` 沒有針對「有真實靜態障礙物在路徑正中間」這種情境特別調過的預設值；③ 值得先跑一次「拿掉錐形障礙物、只留 A/B 兩點」的對照組，確認純粹的 A→B 移動（沒有障礙物阻擋）本身會不會收斂，藉此判斷問題出在「MPC 追目標」還是「MPC 繞障礙物」。

### 8.10 還沒做的部分（NOT RUN，誠實列出）

| 項目 | 狀態 |
|---|---|
| Phase 0：A/B 姿態選定＋獨立 FK 覆核 | ✅ 完成，見 8.1 |
| 共用模組（urdf_fk / frames 擴充 / waypoints / ros_pointcloud）＋單元測試 | ✅ 完成，見 8.2 |
| `reactive.py` --waypoints／新發布 | ✅ 完成＋對真實 Isaac 驗證，見 8.9（`p2p_cone` 情境） |
| 模擬器：overview 發布／新形狀／`--stick none`／`--target-object`／manifest 開放給 dynamic_stick | ✅ 完成＋對真實 Isaac 驗證，見 8.4 |
| `send_joint_cmd.py`：單點命令＋A/B 兩點＋完整 9 步序列 | ✅ 完成＋對真實 Isaac 驗證，見 8.5、8.9 |
| `rerun_dashboard.py`（四視窗介面本體） | ✅ 完成＋對真實 Isaac 驗證，見 8.7 |
| WebRTC 參數轉傳＋viewport probe 顏色檢查泛化＋WebRTC/dashboard 同時跑 | ✅ 完成＋對真實 Isaac 驗證，見 8.8 |
| `run_demo_scenario.sh`＋三個情境（joints/stick/p2p_cone）實際跑過 | ✅ 完成，見 8.9；`p2p_cone` 目前 FAIL，已確認不是逾時設太短，是 MPC 沒有真的收斂（見 8.9 延伸測試） |
| `generate_s2_cube_candidates.py` 泛化＋`run_curobo_grasp_demo.sh --target` | ✅ 完成，見 8.9；`flat_box` 目前生不出候選（真實限制，見 8.9） |
| `static_cylinder`／`grasp` 情境透過 `run_demo_scenario.sh` 實際跑一次 | ❌ NOT RUN（`static_cylinder` 的個別組件在 §8.4 驗證過，只差沒透過這支新腳本跑一次） |
| `tall_block`/`can` 以外的目標（`flat_box` 待候選生成問題解決）真的在 Isaac 裡完整跑完 pick-and-place | ❌ NOT RUN |
| `plan_grasp.py` 的 `--obstacle-manifest`（讓抓取規劃器也讀障礙物 manifest） | ❌ NOT RUN |
| 視窗 1 的體素是否正確跟著障礙物動、dashboard 顯示 reactive 狀態的視覺確認 | ❌ NOT RUN（dashboard 有訂閱這些 topic，但沒有截圖/目視驗證過內容） |
| `p2p_cone` 的 MPC 收斂問題（拿掉錐形障礙物的對照組、調 MPC 收斂/碰撞代價參數） | ❌ NOT RUN，見 8.9 延伸測試最後列的三個懷疑方向 |
