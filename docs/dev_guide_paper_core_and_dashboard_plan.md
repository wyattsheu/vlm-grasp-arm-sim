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
- 修正（在 `references/upstream/mm_system` 本地分支 `fix/place-hold-grip`，**尚未 push**，要由維護者 push 到 nycu-acm/mm_system）：
  - `BaseAction` / `SlowBaseAction.move_arm_to_pose()` 加回 docstring 本來就有寫、但參數被拿掉的 `gripper_width=None`；`None` 維持原本行為（夾爪張開時用，例如 grasp 的接近段）。
  - `PlaceAction` 傳 `gripper_width=self._grasp_close_width`，也就是 grasp 閉合到、回 HOME 時一直鎖住的同一個寬度。
- 模擬器執行的是 `references/upstream/mm_system` **目前 checkout 的分支**；每次的 `report.json` 會寫 `mm_system: {branch, commit, dirty}`，看得出這次跑的是哪一版。切回 `main` 就會重現舊的掉落行為。

**放下時倒下（尚未處理）**：實機 `place.py` 把末端送到「綠墊表面 + `PLACE_HEIGHT_OFFSET_M = 0.07` m」後直接張開。方塊 8 cm 高、夾在上半段，底部離墊子還有幾公分，張開後掉下去就倒了，往 +y 倒半個身長（4 cm）剛好解釋偏移量。實機放品客罐應該有同樣問題，要不要改 offset 或改成「往下降到接觸再放」待決定。

**抓取偵測（有沒有夾到）：原本的程式在哪裡**（使用者說過去有、被封印）

- `mm_system/main_ws/src/mm_actions/mm_actions/arduino_bridge_node.py`：從 Arduino（`/dev/ttyACM0`、115200）讀 5 路 PDMS 觸覺感測電壓，發 `/force_sensor_topic`（`Float32MultiArray`）。
- `mm_actions_node.py` 的 `is_holding_tightly()`：`use_force_grasp=true` 時看最後兩路電壓 ≥ 0.5 V；`false` 時只看「命令寬度 ≤ `grasp_close_width`」，**夾空也回 grasp complete**。
- `adaptive_grasping_node.py`（另一個獨立版本，提供 `/grasp`、`/release` service）：一步 0.5 閉合，任一路電壓比開始時掉超過 2.0 V 就停。訂的是 `/joint_state_feedback`（少一個 s），跟驅動發的 `joint_states_feedback` 對不上，看起來沒被用過。
- 被「封印」的方式：`start_mm_tmux.sh` / `run.sh` / `scripts/run_grasp.sh` 預設 `USE_FORCE_GRASP=false`，不啟動 `arduino_bridge_node`。原因是機器上目前沒裝感測器（GRASP_RUNBOOK §6）。
- 沒有感測器也能用的訊號：驅動的 `joint_states_feedback` 本來就有 `effort[6]`（夾爪力，`grippers_effort/1000`）和量測開口 `position[6]`。夾到東西時，量測開口會停在物體寬度、比命令寬度大（模擬器實測：命令 3.2 cm、量到 3.95 cm）；夾空時兩者會一致。這可以當新的抓取偵測依據，**尚未實作**。模擬器的 `/robot129_sim/piper/joint_states_feedback` 目前只發 position，`effort` 還沒補（Isaac 可以從手指接觸力或關節力算出來）。

同時重現了實機 DEVLOG 已知的問題：[ARM-01]「grasp complete 不代表夾到」、[ARM-09] IK 只約束位置、夾爪姿態自由。

---

## 6. 下一步：這三項要怎麼排序？

三項工作量差異很大（dashboard 中大、WebRTC 腳本小、場景 manifest 中，但場景 manifest 卡在「你要準備 USD 美術資產」這個外部依賴）。建議先做 WebRTC 一鍵腳本（最小、立即有用），再做 dashboard 的 MJPEG 推流部分（最快看到效果），VLM 疊圖跟候選虛影可以晚一點做。場景 manifest 的程式碼部分可以先動工，但美術資產的部分要看你手上有什麼可以用。
