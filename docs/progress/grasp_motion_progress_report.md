# Grasp/Motion 生成模組：進度報告（S0–S5 完成，含真正 MTC 規劃執行與真實 VLM）

日期：2026-09-18 起（Asia/Taipei），本次修訂 2026-09-20。對應計畫：
`grasp_motion_research_plan_20260918.md`、`grasp_motion_remote_runbook_20260918.md`
（見本地 handoff `grasp_motion_handoff_20260918/`）。本報告只記錄本輪**實際在遠端
`robot129_pro6000_sim_20260913` 執行並驗證過**的結果；所有數字均來自真實 Isaac PhysX
模擬與真實 ROS 2 動作伺服器，不是預先寫死的展示。

## 這次做到什麼程度（一句話）

**MoveIt Task Constructor 對著即時 Isaac 機器人狀態、從外部候選 JSON 讀取多個抓取候選、
逐一嘗試規劃出完整取放任務（IK、碰撞檢查、OMPL/CartesianPath 都是真的），規劃結果經由
FollowJointTrajectory adapter 送進同一個 live Isaac 場景執行，平行夾爪從桌面真正夾起
方塊（真實 PhysX 接觸、無 kinematic attach）、搬運、精準放下（`tools/
run_grasp_motion_demo.sh` 一行指令即可重現）。候選來源可以是離線幾何、也可以是對真實
腕上相機影像跑真實本地 VLM 產生的候選（S4），且已經對一個非立方體的真實雙色物件驗證過
（S5 延伸）。**

## 對照 ZeroDex §3.4 的模組分解

| ZeroDex 模組 | 本輪狀態 |
|---|---|
| Affordance region → grasp candidates | **完成，對 live Isaac 驗證**：`research/src/mpg/grasp_candidates.py`（候選生成）＋`affordance_region.py`（VLM bbox → 3D 任務區域）；S4 對真實相機影像跑過真實本地 VLM，S5 對真實雙色鎚形物驗證過 |
| Collision-aware placement refinement | **完成（簡化版）**：place 側「原點＋四鄰居」xy 偏移搜尋，見「place 側候選化」一節；不是論文式 12 的完整任務區域搜尋 |
| Motion generator 對每個 candidate 求解 (𝒯robot, η) | **完成，對 live Isaac 驗證**：MTC 讀外部候選 JSON，逐一嘗試（grasp × place 巢狀迴圈），選第一個可行的；`tools/run_grasp_motion_demo.sh` 一行指令跑通規劃＋真實執行全鏈路 |
| 選第一個可行 grasp-motion pair | **完成，對 live Isaac 驗證**：見「S3→S2 收尾」與「place 側候選化」兩節，含刻意失敗案例證明 fallthrough 邏輯正確 |

完整技術細節、每一步的真實執行證據、以及誠實記錄的範圍限制，見下方各章節（依時間順序
S0→S5，最新的修訂與真實 bug 修正記錄在文件最後「2026-09-20 修訂」一節）。

## S0：現況盤點與可達性分析 — 完成

- `docs/progress/grasp_motion_s0_inventory.md`、`grasp_motion_interface_map.yaml`：控制鏈、MoveIt/MTC、既有 grasp demo 逐項附 file:line 證據。
- `tools/analyze_top_down_reach.py`：純 numpy FK 掃描，**用 `out/lesson_06/gripper_probe.json` 的實測 HOME 姿態回代驗證，誤差 <1e-6 m**，證明 FK 正確；由上往下抓取在 z=0.0175–0.15 m 皆有解（`out/grasp_motion/s0/reach_scan.json`）。
- 發現並記錄三個會改變設計的問題：`tcp` frame 偏移錯誤（0.180 m 應為 ~0.125 m）、joint5 限制對高處由上往下抓的影響、WebRTC port 49100 被你自己的 parcel-forge viewer 占用。

## S1：控制鏈 — 完成，含即時 Isaac 驗證

### Runner 修改（`sim/scripts/run_robot129_ros_webrtc.py`）

新增 `--scene pick_place`（真實重力/摩擦動態方塊＋接觸感測器）、接受 MoveIt 風格的
`time_from_start=0` 首點、以 sim 時間線性/三次插補取代原本牆鐘 smoothstep、新增
`objects/target_cube/pose`、`contacts/link7`、`contacts/link8`、`joint_command`、
`trajectory_events`、`scene_state` topic，以及 `reset_scene`（Trigger）、`recording`
（SetBool）service。`--scene marker`（預設）行為與修改前逐位元組相同，原有 regression
（`out/lesson_08`、`out/lesson_09` 等既有報告）未被觸碰。

**即時驗證**（非模擬預期，實際執行）：
- 啟動 `tools/start_robot129_grasp_sim.sh --scene pick_place`，headless、不占用 port 49100。
- 方塊在真實重力下落穩定於 z=0.017499 m（設計值 0.0175 m，誤差 <0.1 mm）。
- `reset_scene`、`recording`（含 ffmpeg 背景編碼出 mp4）皆呼叫成功。
- **量得這台共用 GPU 上的實際即時倍率約 0.166–0.169**（120 Hz 物理步頻下，一段 3 秒與 6 秒的軌跡分別花費 18.05 s、35.58 s 完成），遠低於名目 120 Hz；已記錄進 `robot129_sim_execution/robot_model.py` 並用於 adapter 逾時計算，避免用「近乎即時」的錯誤假設設計 timeout。

### 新套件 `ros2_ws/src/robot129_sim_execution`（FollowJointTrajectory adapter）

獨立 ROS 2 節點（不塞進 Isaac 程序），提供 `/robot129_sim/{arm,gripper}_controller/follow_joint_trajectory`。

- 依名稱重新排序關節（容忍任意順序）、拒絕未知/重複/缺漏/非有限值/超限位置。
- busy 時拒絕新 goal（不搶占）；requires this adapter be the sole writer on the command topic。
- cancel 會送出「停在實測位置」的 hold 命令，確認後才回報 CANCELED。
- 以 steady clock 偵測 JointState 斷流（stale feedback）而中止。
- 逾時計算採用**實測即時倍率**而非固定秒數：`timeout = duration / min_realtime_factor + margin`。
- 夾爪「卡在物體上停住」（因接觸力達門檻但未達目標開口）視為明確成功，另記錄 `stalled_on_object`，不是靜默 PASS。
- 每個 goal 記錄 goal id、scene revision、robot state timestamp、結果到
  `<log_dir 參數>/events.jsonl`（預設相對路徑 `out/grasp_motion/adapter/events.jsonl`，
  以啟動時的工作目錄為準；正式 launch 建議用絕對路徑，見 `config/adapter_params.yaml`）。

**驗收結果**：
1. `ros2_ws/src/robot129_sim_execution/test/`：9 個 pytest 整合測試，對一個假 runner
   （不需 GPU/Isaac）跑在隔離 ROS domain 219，**9/9 通過**：正常執行、關節順序打亂、
   未知/缺漏/超限關節拒絕、busy 拒絕、cancel+hold、逾時中止、stale feedback 中止。
2. **對真正 live Isaac（domain 129）的直接驗證**（非模擬）：
   - `ros2 action send_goal` 送出真實關節目標 → `error_code=0 error_string=OK`，
     `status=SUCCEEDED`。
   - 未知關節名稱 → `Goal was rejected`。
   - 兩個 goal 背靠背送出 → 第二個立即 `Goal was rejected`（busy policy 生效）；
     第一個之後在 `events.jsonl` 記錄 `STARTED`→`SUCCEEDED`。
   - 用絕對路徑 `log_dir` 重跑一次單一 goal，確認 `out/grasp_motion/adapter/events.jsonl`
     確實落在文件所述位置且內容正確；同時確認 `scene_state` 的 transient-local QoS
     修正後，`scene_revision` 從先前的 `null` 變成正確的 `0`（見下方「本輪順便修的兩個
     bug」）。
3. 全部透過一般 `colcon build`（非 `--symlink-install`，因為目前環境的 setuptools 版本
   移除了舊版 editable-install 選項，改走一般 build 才能過）。

## S2 前置：手動 waypoint 驗證控制鏈（在真正上 MTC 之前）

`tools/verify_grasp_motion_manual_waypoints.py`：用一次性數值 IK（阻尼最小平方法，
4 個約束：x/y/z 與逼近軸朝下）求解 S0 選定場景（方塊 (0.32,0,0.0175)、放置點
(0.27,-0.12,0.0175)）的 pregrasp／grasp／place_lift／place_grasp 五個姿態，**直接送
raw JointTrajectory**（不經過 adapter，也不經過 MoveIt/MTC）完成完整
approach→descend→close→lift→transit→place→open→retreat→home 序列。

**實測結果**（`out/grasp_motion/manual_waypoints/20260918T181828/report.json`，
影片 `out/grasp_motion/sessions/run_0002/video.mp4`，12.7 秒）：`kinematic_attachment=false`、
抬升 0.0562 m、放置 xy 誤差 0.021 mm、`trajectory_success`／`physical_grasp_success`／
`task_success` 皆 `true`，重複兩次結果一致。這組關節角是為這個特定方塊位置手算的，換位置
要重新解 IK，**不是** ZeroDex 定義的「候選生成＋規劃器」；它的作用是在花力氣修 MTC 之前，
先證明控制鏈（t=0 接受、sim 時間插補、真實動態方塊）本身沒問題。

## S2：MoveIt Task Constructor 規劃並執行真正取放 — 完成

新 C++ 節點 `ros2_ws/src/robot129_tasks/src/robot129_mtc_pick_place.cpp`：
CurrentState → clamp-to-bounds → add scene objects → open gripper → Connect(OMPL
RRTConnect) → SerialContainer{approach(CartesianPath) → ComputeIK(已知候選姿態)} →
close gripper → attach → lift(CartesianPath) → Connect → SerialContainer{lower
(CartesianPath) → ComputeIK(放置姿態)} → open → detach → retreat(CartesianPath)。
`CurrentState` 讀的是**真正 live Isaac 的 `/robot129_sim/joint_states`**（node 跑在
`/robot129_sim` namespace 下，且另外啟動一個 plan-only 的 `move_group` 供
`CurrentState` 取得初始 `/get_planning_scene`），不是假資料。規劃完成後把
solution 的每段子軌跡（joint_names＋時間化 waypoint）匯出成 JSON
（`out/grasp_motion/mtc_pick_place.json`），因為這個工作區的 MTC 只建了 C++、沒建
`ExecuteTaskSolution` capability（見 S0 盤點），所以執行走「匯出→Python 執行器逐段送
adapter」，不是靠 move_group 直接執行。

新 Python 執行器 `robot129_sim_execution/run_grasp_motion.py`：讀取匯出的 JSON，過濾掉
真正零位移的 no-op 段（例如 clamp 後夾爪已經很接近目標，MTC 規劃出單點 t=0 軌跡），把
其餘每段依序送進 S1 的 adapter，並在段落之間用真實方塊姿態（不是規劃值）判定
`physical_grasp_success`（夾爪閉合後量到的最高點）與 `task_success`（最終停放位置）。

**實測結果**（`out/grasp_motion/mtc_pick_place.json` 規劃報告 + `out/grasp_motion/mtc_execution/20260918T215409/report.json` 執行報告，
影片 `out/grasp_motion/sessions/run_0003/video.mp4`，7.8 秒）：

| 檢查項 | 結果 |
|---|---|
| MTC 規劃 | `PASS`，1 個解，19 段子軌跡（含 approach/grasp-IK/close/attach/lift/connect/lower/place-IK/open/detach/retreat，實際運動段 8 段 + 1 個零位移 no-op） |
| 執行段落 | 8/8 `SUCCEEDED`（含一段 gripper `stalled_on_object`：關節誤差 0.0134 m 超過容忍度，但 link7/link8 接觸力各 ~10 N，判定確實夾住物體，非靜默 PASS） |
| `kinematic_attachment` | `false` |
| 抬升高度 | 0.0602 m（門檻 0.03 m） |
| 最終放置位置 | (0.269967, -0.119998, 0.017500) vs 目標 (0.27, -0.12, 0.0175)，xy 誤差 <0.05 mm，z 誤差 ~1e-7 m |
| `trajectory_success` / `physical_grasp_success` / `task_success` | 皆 `true` |
| 總狀態 | `PASS` |

### 過程中遇到並解掉的問題（誠實記錄，不是一次就過）

1. **建置環境缺 `-lfmt`／`-llttng-ust` link 路徑**：這台機器的 conda ROS 環境有對應的
   `.so`，只是連結器搜尋路徑沒指到那裡；在 `robot129_tasks/CMakeLists.txt` 加
   `link_directories($ENV{CONDA_PREFIX}/lib)`，只影響這個套件，沒動共用環境。
2. **執行期同樣缺 library path**：另外要在啟動指令加
   `LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"`。
3. **`FixedCartesianPoses` 需要 `setMonitoredStage`**：一開始沒設，`task.init()` 直接報
   「`target_pose` declared but undefined」；還需要在 `ComputeIK` 上加
   `configureInitFrom(Stage::INTERFACE, {"target_pose"})`（官方教學範例裡有、但這兩行很
   容易漏)。
4. **`CurrentState` 需要一個 plan-only `move_group` 陪跑**：不然它去要
   `/get_planning_scene` 會 timeout。這個 `move_group` 只是輔助基礎設施，不用來執行。
5. **joint8（mimic joint）在真實 Isaac 下會漂出宣告邊界**：`joint8` 是 URDF 裡
   `multiplier=-1` 的 joint7 mimic，但 runner 是用**獨立的 PD 伺服**驅動它，不是單純數學
   算出來的，所以真實量到的值偶爾會比 -0.035 這個下限還負一點點。這不是浮點誤差，是
   PhysX 沒有完美遵守 `NewtonMimicAPI`（S0 盤點時就標了 UNVERIFIED，這次算是被實測證實）。
   解法是在 `ModifyPlanningScene` 用 callback 夾住讀到的關節值——但只能夾**非 mimic 的來源
   關節**（joint7），因為對 mimic joint 自己寫值會被 `state.update()` 重新從 joint7 算過去
   蓋掉，等於白夾。
6. **`FixedCartesianPoses::compute()` 是從「被監控 stage」的結束場景 diff 出來的，不是從
   上一個 sibling stage**：一開始兩個候選都監控 `CurrentState`，結果 grasp／place 分支各自
   重新從最原始（未夾過邊界）的場景長出來，把前面每一次夾邊界的修正都蓋掉了——症狀是
   同一個關節超界錯誤在完全不同的後續 stage 反覆出現。改成 grasp 候選監控
   「open gripper 後夾過邊界」的 stage、place 候選監控 `lift object` stage 後解決。
7. **support_floor 與 base_link／target_cube 在同一個 z=0 平面上會判定為碰撞**：跟
   `verify_grasp_motion_manual_waypoints.py` 遇到的道理一樣，真實場景本來就是貼著放，加
   `allowCollisions` 排除這兩對。
8. **執行器把 comment 誤判成 stage 名稱**：MTC solution 匯出的 `comment` 欄位其實是
   solver 自己的訊息（如 "Solution generated by RRTConnect"），不是 stage 名稱（要有
   `Introspection` 才能還原 stage 名稱，本輪匯出時沒接）。第一次執行成功了（抬升 6cm、精準
   放置）但我自己的成功判定邏輯誤判為 FAIL，因為它在找 comment 裡有沒有 "lift" 字樣。改成
   直接看夾爪閉合後量到的方塊最高點，不依賴文字比對。
9. **`tools/stop_robot129_grasp_sim.sh` 偶爾殺不掉 Isaac 子行程**：見下方「本輪順便修的
   bug」。

以上第 5–8 點都是「只有對著真正 live Isaac 測才會發現」的問題；純規劃（無 Isaac）或用假
runner 的單元測試都不會踩到，這也是為什麼本輪堅持每一步都要對著真正的模擬驗證，而不是
規劃完就宣稱完成。

## 本輪順便修的兩個 bug（對真正 live Isaac 驗證時才發現）

1. **`scene_state` QoS 不匹配**：runner 用 transient-local（latched）發佈，但 adapter 原本
   用預設 volatile 訂閱，導致沒有 reset 過的情況下 adapter 永遠收不到任何 `scene_state`
   （`scene_revision` 恆為 `null`）。改成同樣 transient-local 後，重跑確認 `scene_revision`
   正確變成 `0`。這是「訂閱端 QoS 要跟發佈端 durability 對齊」的教科書案例，只有真的對 live
   系統測才會發現（假 runner 測試沒有踩到，因為測試裡我用的是預設 QoS 兩邊都一致）。
2. **`tools/stop_robot129_grasp_sim.sh` 有時殺不掉 Isaac 子行程**：`nohup setsid uv run
   ...` 記錄的 PID 是 `uv run` 這層，但 IsaacLab/Kit 底下的實際 GPU 行程有時會自己開一個
   新的 process group（規避訊號），導致對原 process group 送 SIGTERM 成功但主行程沒死。
   本輪已改成同時用 `pkill -f` 比對指令列當第二條殺法，並在腳本結束前主動確認真的沒有殘留
   行程才回報 `stopped`；15 秒後還沒死會補一次 SIGKILL。已用「先啟動、再驗證確實停止」
   重新測過。

## S3：幾何抓取候選生成器 — 完成（離線），已對真實機器人驗證

新模組 `research/src/mpg/grasp_candidates.py`（純 numpy，不依賴 ROS/Isaac）：對輸入的物體
點雲（GT 幾何或分割點雲）＋可選的任務允許接觸區域遮罩，繞世界垂直軸取樣 yaw（預設 8 個），
每個 yaw 各自：算出沿該閉合軸的投影寬度（超過夾爪最大開口 → `WIDTH_EXCEEDED`，太窄 →
`WIDTH_TOO_NARROW`）、夾點離桌面淨空（不足 → `TABLE_CLEARANCE`）、投影兩端點雲密度是否足夠
（不足 → `UNKNOWN_GEOMETRY`）、兩端中心連線是否大致對穿過抓取中心（角度過大 →
`NON_ANTIPODAL_EDGE_CONTACT`）。全部候選（含被拒絕的）都會回傳並附理由，不悄悄丟棄。輸出
姿態用「pinch point」慣例（與 `robot129_mtc_pick_place.cpp` 的 MTC ik_frame offset 完全一致，
已對真實接觸範圍交叉驗證過）。`research/src/mpg/grasp_contract.py` 負責 JSON 匯出／驗證
（欄位對應研究計畫 §8），`research/configs/grasp_generation.yaml` 放夾爪與取樣參數。

**單元測試**：`research/tests/test_grasp_candidates.py`，25 個測試，涵蓋 SE(3)
乘法／inverse round-trip、quaternion 合成順序、90°/180° 旋轉正確性、寬度篩選、桌面淨空篩選、
空任務區域、偏心任務區域會位移抓取中心、JSON contract 的 accepted⇔無理由 不變量、寫入讀回。

**對照真實機器人驗證**（不是只在合成資料上測試）：用 S2 已知方塊的真實幾何（0.035 m 立方體，
中心 (0.32, 0, 0.0175)）跑這個生成器，**最佳候選的位置與 MTC 規劃並在 live Isaac 執行成功
的真實抓取姿態完全吻合**（(0.32, -0.0, 0.0175) vs (0.32, 0, 0.0175)），方向也落在同一組
（對稱立方體下）幾何等價的姿態上。過程中連續抓出兩個真的錯誤，只有對照已驗證的真實結果才
發現得到：
1. **夾點高度預設抓「點雲最高點」，應該抓「垂直中點」**：只餵立方體頂面點雲時，生成器把
   夾點放在 z=0.035（頂面），但真實成功的抓取是在 z=0.0175（立方體中心）。這是因為平行夾爪
   包夾一個比手指短的物體時，接觸發生在物體中段，不是頂面。加了 `grasp_height_mode` 參數
   （預設 `region_midpoint`，用點雲最低與最高點的中點；`region_top` 保留給「只看得到頂面」
   的單視角情境，但那種情境還需要另外估物體高度，本輪尚未接）。
2. **候選評分方向寫反**：一開始的分數公式在數學上等於「離夾爪最大開口越近分數越好」，跟
   註解寫的「應該偏好開口餘裕大的候選」剛好相反，會把最貼近極限、風險最高的候選排第一。
   改成 `score = opening_width_m / gripper_max_opening_m`，讓餘裕最大的候選排最前面；加了
   `test_best_candidate_prefers_most_closing_margin_not_least` 回歸測試防止再犯。

**範圍限制（誠實記錄）**：目前是**離線驗證**——生成器產生的候選姿態經人工比對，確認與已
用 MTC＋adapter＋live Isaac 走完整套物理取放的姿態一致，但**還沒有把多候選接回
`robot129_mtc_pick_place.cpp`**（目前那個 C++ 節點裡的候選姿態仍是寫死的單一組）。下一步
是讓 C++ 節點讀外部候選 JSON、對多個候選各跑一次 task、選第一個可行的（ZeroDex §3.4
的 (𝒯robot, η) 迴圈），才算把 S3 的產出接回 S2 的執行鏈。

### S3 延伸：VLM 抓取區域（affordance）→ 3D 任務區域遮罩 — 完成（離線，合成資料驗證）

這部分是接續 2026-09-19 的討論：既有單相機管線是「RGB → VLM 出一個抓取點 → 查深度」，
這裡把 ZeroDex 附錄 D.2「Grasp Affordance」那個 prompt（論文原文，不是重寫的近似版）接進
既有的 grounding pipeline，讓 VLM 多輸出一個「允許碰的區域」框，再把這個框轉成
`grasp_candidates.py` 要的 `task_region_mask`，形成完整鏈路：**VLM 出框 → 深度反投影 →
物體遮罩與框取交集 → 3D 任務區域 → 抓取候選生成**。單一視角，沒有論文的多視角投票（eq.
10-11），這點在 prompt 檔跟本檔都有明講。

**新增檔案**：
- `research/prompts/grasp_affordance.txt`：論文附錄 D.2「Grasp Affordance」原文改寫，附完整
  provenance header（來源行號、來源檔 sha256）。跟既有 `stage_b.txt` 同樣的「刻意偏離」寫法：
  原文沒有「找不到就說找不到」的退路，這裡照 `stage_b.txt` 已有的規則加了
  `status="unavailable"` + `reason_codes`，VLM 找不到安全接觸區時不用硬猜一個框。
- `research/src/mpg/schema.py`：新增 `AffordanceRegion` dataclass 與
  `parse_grasp_affordance_response()`，跟 `LocatedStep`／`parse_stage_b_response` 用同一套
  「localized/unavailable + reason_codes」驗證邏輯與 `REASON_CODES` 詞彙表，bbox 額外驗證
  `y1<y2` 且 `x1<x2`。
- `research/src/mpg/grounding.py`：新增 `run_grasp_affordance()`，跟 `run_stage_a`／
  `run_stage_b` 同一套「格式錯誤重試一次、正確的 abstention 不重試」流程，共用同一個
  `BaseBackend`（快取／預算／log 都不用重寫）。
- `research/src/mpg/lifting.py`：新增 `deproject_region()`，是既有單點 `deproject()` 的向量化
  版本，同一套 pinhole 公式，多一個「哪些像素要反投影」的布林遮罩參數。
- `research/src/mpg/affordance_region.py`（新模組）：`affordance_bbox_to_pixel_mask()` 把
  `AffordanceRegion` 的正規化 bbox 轉成像素遮罩；`object_points_and_task_region_mask()` 把
  「物體遮罩」跟「VLM 框」**取交集**（不是只信框本身——單視角一個框常常會連背景一起框進去，
  這個交集邏輯就是 2026-09-19 那次討論裡提的建議），輸出整個物體的 3D 點雲＋一個布林遮罩，
  剛好是 `generate_grasp_candidates(points_xyz, task_region_mask=...)` 要的格式。
  **重要限制寫在 module docstring 裡**：`deproject_region()` 回傳的是「相機座標系」，
  `generate_grasp_candidates` 假設的是「世界／base 座標系」（它的由上而下逼近軸就是世界 -Z），
  這個模組**不會**自動幫你做相機到世界的轉換——呼叫端要自己用場景的
  `T_base_camera`（`scene_bundle.transform_matrix`）轉過去，沒轉會產生看起來能跑但數字全錯
  的結果。

**測試**：`research/tests/test_schema.py`（9 個新測試，parser 的合法／不合法輸入）、
`test_grounding.py`（5 個新測試，含「正確的 unavailable 不能重試」）、`test_lifting.py`（6 個
新測試，`deproject_region` 跟既有 `deproject` 單點結果比對一致）、新檔案
`test_affordance_region.py`（9 個測試）。最後這個檔案裡有一個**端到端測試**
（`test_full_chain_prefers_the_affordance_restricted_region`）：造一個合成的「瓶身＋瓶蓋」
深度影像，VLM 框只圈瓶身，整條鏈路（反投影→交集→世界座標轉換→
`generate_grasp_candidates`）跑完後，斷言選中的候選的抓取中心確實落在瓶身的 y 範圍內、不
在瓶蓋範圍——不是用文字斷言「這樣應該可以」，是真的用幾何算過一次。全部 research 單元測試
現在是 **164 個**（原 108 ＋ S3 候選生成器 25 ＋ 這次 31 個），全數通過，含既有測試無回歸。

**還沒做的部分（誠實記錄）**：這條鏈路目前只在合成資料上驗證過，**還沒有接上真正的 VLM
呼叫**（`run_grasp_affordance` 本身測試用的是假 backend）、**也還沒有接上腕上相機的真實
RGB-D／物體遮罩**——物體遮罩（`object_pixel_mask`）目前假設呼叫端已經有（GT mask 或顏色分割
都行，這個模組不在意來源），這部分是 S4「接既有多點」排程要做的事，不是這次的範圍。

### S3→S2 收尾：多候選接回 MTC，對 live Isaac 完成驗證 — 完成

前一輪 S3 的「範圍限制」寫的是「候選生成器產出的姿態只經人工比對，沒有真正接回
`robot129_mtc_pick_place.cpp`（那個 C++ 節點裡的候選姿態還是寫死的單一組）」。這輪把這個缺口
補上，對應 ZeroDex §3.4「對候選集合 G 裡每個 g 各跑一次 f_motion，取第一個可行的
(𝒯robot, η)」那個迴圈。

**改動內容**（`ros2_ws/src/robot129_tasks/src/robot129_mtc_pick_place.cpp`）：
- 把原本一次性的 stage-graph 建構搬進 `buildAndPlan()` 函式，接受任意 grasp_pose，每個候選各自
  用一個**全新的** `mtc::Task`（不重複使用同一個 Task 物件，避免上一個候選失敗留下的 planning
  scene 狀態污染下一個候選）。
- 新增一個約 150 行、範圍限定在 `grasp_contract.py` 輸出格式的最小遞迴下降 JSON parser
  （`namespace tinyjson`，只讀不寫，跟既有的 `jsonEscape()` 手寫寫入器一樣，是刻意不引入新
  函式庫依賴的選擇）。讀取 `candidates_path` ROS 參數指到的 JSON，只取 `accepted=true` 的列，
  依 `score` **升冪**排序（`grasp_candidates.py` 自己的慣例：分數越低代表夾爪開口用得越少、
  留的閉合餘裕越多，即越安全——這點程式碼本身的註解已經寫明；第一版接線時我把排序方向寫反了
  ＜降冪＞，跑起來仍然「能動」但會優先挑最沒有餘裕的候選，靠重新核對
  `grasp_candidates.py` 裡「Lower score is better, and sorted ascending below」那行註解才抓到，
  已修正）。
- 新增 `candidates_path`（預設空字串）與 `max_candidates`（預設 8）兩個 ROS 參數，
  `mtc_pick_place_sim.launch.py` 對應加了兩個 `DeclareLaunchArgument`。**預設空字串時完全維持
  原本 S2 的單一寫死候選行為**（regression-safe，沒傳 `candidates_path` 的既有呼叫方式不受影響）。
- 每個候選的嘗試結果（`candidate_id`、`score`、`initialized`、`planned`、`planned_solutions`、
  `failures`）都寫進 report JSON 的新欄位 `candidate_attempts`（不只贏家那筆），連同
  `chosen_candidate_id`、`candidates_considered`、`candidates_load_error`。這份逐候選紀錄本身
  就是「機器為什麼選這個動作」的可查詢紀錄，也是之後如果要做候選視覺化（例如把每個嘗試過的
  候選姿態畫成疊加影像）現成的資料來源，不用另外再插測探針。

**新增檔案** `tools/generate_s2_cube_candidates.py`：對 S2 pick_place 場景已知的 `target_cube`
幾何（中心 (0.32,0,0.0175)、邊長 0.035m，跟 `run_robot129_ros_webrtc.py`／C++ 節點裡寫死的場景
一致）取樣立方體表面點雲，餵給 `generate_grasp_candidates()`，用 `research/configs/
grasp_generation.yaml` 的同一組夾爪參數，輸出到
`out/grasp_motion/candidates/s2_cube_candidates.json`。這是幾何 ground truth，不是感知結果——
「點雲從哪來」仍然是 S4 的範圍。

**對 live Isaac 的三個驗證跑**（`ros2 launch robot129_tasks mtc_pick_place_sim.launch.py`，跟
既有 S2 驗證同一套 launch，只是多傳 `candidates_path`）：

| 情境 | candidates_path | 結果 |
|---|---|---|
| 迴歸：不傳 candidates_path | （空） | `chosen_candidate=hardcoded_single`，PASS，跟改動前行為一致 |
| 8 個真實候選，全部可行 | `s2_cube_candidates.json` | 第一次嘗試就選中分數最低的 `G004`（score=0.5072），PASS |
| 刻意插入一個「分數最低但不可達」的假候選在最前面 | 同上 + 手動插入 `G_UNREACHABLE`（score=-1.0，位置 (5,5,5) 遠超工作空間） | `G_UNREACHABLE` 的 IK/規劃如預期失敗，迴圈正確往下嘗試，第二個候選 `G004` 成功，`attempts=2/9`，PASS |

第三個情境是刻意設計的失敗案例，用來證明「選第一個可行候選」這個迴圈邏輯真的在起作用，不是
剛好第一個候選永遠都行才看起來像有在做選擇。三次都是對 live Isaac 的真實 MTC 規劃（非
mock/fake joint states），不是單元測試。

**範圍限制（誠實記錄）**：place 目標仍是原本寫死的單一姿態，只有 grasp 側走候選迴圈；
grasp candidate 的挑選邏輯是「第一個 IK/規劃成功」，沒有像論文式 12 那樣做「碰撞感知的最近
合法位置搜尋」；`max_candidates=8` 只是搬移既有計畫書寫的數字，沒有另外調過。

### 候選決策視覺化：RViz MarkerArray — 完成，對 live Isaac 驗證

延續上面「S3→S2 收尾」新增的 `candidate_attempts` 決策紀錄，這輪把它畫成 RViz 看得到的東西：
每個嘗試過的候選一組 ARROW（畫在候選姿態上，箭頭方向＝該候選的夾爪朝向，指的是
`generate_grasp_candidates()` 掃描 yaw 時實際用的那個姿態，不是隨便擺的示意）＋一個
TEXT_VIEW_FACING 文字標籤（候選 id、score、CHOSEN／REJECTED、若失敗則附第一條 MTC 失敗原因）。
綠色＝被選中的候選（`task.plan()` 成功的那個），紅色＝在它之前被嘗試過、失敗的候選。

**改動內容**（同一個檔案 `ros2_ws/src/robot129_tasks/src/robot129_mtc_pick_place.cpp`）：
- 新增 `buildCandidateMarkers()`：把 `(candidate_id, score, grasp_pose, chosen, failure_text)` 的
  清單轉成 `visualization_msgs::msg::MarkerArray`，每個候選輸出兩個 marker（箭頭＋文字），最前面
  補一個 `DELETEALL` marker，避免上一次執行留下的候選殘留在 RViz 畫面上。
- `NamedAttempt` 結構新增 `grasp_pose` 欄位，兩個既有的 `attempts.push_back(...)` 呼叫點都補上，
  這樣候選迴圈本來就在算的姿態不用重算一次就能拿來畫圖。
- 在寫完 report JSON 之後、`executor.cancel()` 之前，用 `transient_local + reliable` QoS 建立一個
  `grasp_candidate_markers` publisher（namespace 下實際主題是
  `/robot129_sim/grasp_candidate_markers`），發布整個 MarkerArray，並 `rclcpp::sleep_for(500ms)`
  讓訊息真的有時間送出去——這個節點規劃完就會結束（不是常駐節點），要留一點時間讓
  transient_local 的訊息確實進到 DDS，不然還沒送出程序就已經關掉。
- `CMakeLists.txt` / `package.xml` 新增 `visualization_msgs` 依賴。

**為什麼是 transient_local**：這個節點的設計本來就是「規劃完就退出」，不是常駐服務；如果你在
它發布之後才打開 RViz、新增 MarkerArray 顯示，一般 QoS（volatile）會直接收不到，因為節點已經
關閉了。`transient_local` 讓最後一則訊息在 publisher 存活期間可以被「晚到」的訂閱者收到——但要
注意 RViz 的 MarkerArray 顯示預設 QoS 通常是 Reliable/Volatile，**要手動把 Durability Policy
改成 Transient Local** 訂閱端才對得上，這點寫在下面的操作步驟裡。

**驗證方式（誠實記錄：這是我唯一能做的驗證，沒有畫面截圖）**：這台機器上沒有能跑 RViz GUI
的顯示環境，所以沒辦法截圖確認畫面渲染是否正確（UNVERIFIED：實際在 RViz 視窗裡長什麼樣子）。
但用一支獨立的 `rclpy` 訂閱腳本（跟 RViz 訂閱同一個 topic、同一套 QoS：depth=1、
TRANSIENT_LOCAL、RELIABLE）對 live Isaac 跑了一次帶假失敗候選的完整流程，證實訊息確實被送出、
確實可以被外部訂閱者收到、且內容正確：

```
received MarkerArray with 5 markers
  ns=grasp_candidates id=0 type=0 action=3 ...                      # DELETEALL
  ns=grasp_candidates id=0 type=0 action=0 pos=(5.0000,5.0000,5.0000) color=(0.85,0.15,0.15,0.90)   # 紅色箭頭：G_UNREACHABLE
  ns=grasp_candidates_labels id=1 ... text='G_UNREACHABLE score=-1.000\nREJECTED'
  ns=grasp_candidates id=2 type=0 action=0 pos=(0.3200,-0.0000,0.0175) color=(0.10,0.75,0.15,0.90)  # 綠色箭頭：G004
  ns=grasp_candidates_labels id=3 ... text='G004 score=0.507\nCHOSEN'
```

位置、顏色（紅＝拒絕在 (5,5,5) 那個刻意設計的失敗候選、綠＝選中的 `G004` 在真實方塊位置）、文字
都跟預期一致。這證明「訊息有正確送達＋內容正確」，但「RViz 畫出來是否符合你想要的視覺效果」
仍待你自己開 RViz 確認。

**怎麼看（操作步驟）**：
1. 依平常方式啟動 live Isaac 場景與跑 MTC（見上面 S2 的啟動方式），加上
   `candidates_path:=<你的候選 JSON>` 觸發多候選規劃。
2. 開一個新終端機，`ros2 run rviz2 rviz2`（要先 source 這個 workspace 的 ROS 環境，同 S2 的做法）。
3. Fixed Frame 設成 `world`。
4. 左下角「Add」→「By display type」→ 選 `MarkerArray`，Topic 填
   `/robot129_sim/grasp_candidate_markers`。
5. **關鍵一步**：在該 MarkerArray 顯示的設定裡，把 `Durability Policy` 從預設的 `Volatile` 改成
   `Transient Local`（不改的話，如果 RViz 是在 MTC 節點發布「之後」才訂閱，會看不到任何東西，
   這正是這個 topic 用 transient_local 而不是預設 QoS 的原因）。
6. 執行 MTC 規劃（步驟 1），完成後 RViz 裡應該會出現每個候選一組箭頭＋文字標籤。

**範圍限制（誠實記錄，寫作當下）**：只有 grasp 候選有畫，place 姿態沒有（因為 place 側當時還沒
走候選迴圈）；箭頭長度／樣式是固定值，沒有依候選分數或失敗類型做視覺上的額外區分（例如 IK
失敗跟規劃逾時目前都是同樣的紅色箭頭，差異只反映在文字標籤裡）；沒有做「Isaac 場景內半透明
虛影機械臂」（ZeroDex 討論裡提過的方案一），那個仍然只是構想，還沒實作。**下面「place 側候選化」
完成後，place 姿態也已經一起畫進同一個 MarkerArray（獨立 namespace `place_candidates`）。**

### place 側候選化 ＋ 碰撞感知位置修正（ZeroDex 式 12 簡化版）— 完成，對 live Isaac 驗證

前面「S3→S2 收尾」只讓 grasp 側走候選迴圈，place 目標仍是單一寫死姿態。這輪把 place 側也接上
一個簡化版的「碰撞感知最近合法位置搜尋」（對應論文式 12），並讓兩側的迴圈巢狀在一起，符合
ZeroDex §3.4「對每個 grasp candidate 都跑一次完整 motion generator，內部處理 place 的位置修正」
的語意，而不是把兩者當成互相獨立的迴圈。

**設計（誠實記錄：這是簡化版，不是論文原始方法）**：`PlaceOffsetCandidate` 定義一組固定的
「原點＋上下左右四鄰居」xy 偏移網格（預設步長 1.5cm，`--place_offset_step_m` 可調），對每一個
grasp candidate，依「原點優先」的順序嘗試這五個 place 偏移，直到整個任務（grasp ＋ place 都要
成功）規劃成功，或五個都試完。跟 grasp 側一樣，**用完整 MTC task 重新規劃（含真實碰撞檢查）
當作「這個位置合不合法」的判斷依據**，而不是另外寫一個獨立的碰撞查詢工具——這個專案裡本來就
沒有這樣的工具，額外做一個超出這次的範圍。**這不是**論文式 12 真正的作法：沒有依任務可抓區域
去搜尋、沒有隨失敗自動擴大搜尋半徑、也不會區分「這裡真的沒地方放」跟「IK 剛好在別的原因失敗」，
只是問「整個任務解不解得開」。

**巢狀迴圈**：外層跑 grasp candidate（分數由低到高，即最安全的先試），內層對每個 grasp candidate
跑 place 偏移（原點優先）；第一個 (grasp, place) 都成功的組合勝出。每個 (grasp, place) 嘗試都記錄
進 report JSON 的 `candidate_attempts`，欄位新增 `grasp_candidate_id`／`place_candidate_id`
分開記錄（`candidate_id` 保留組合字串如 `"G004+place_x+"` 方便查找），另外新增
`chosen_grasp_candidate_id`／`chosen_place_candidate_id`。

**對 live Isaac 的驗證跑**（四個情境，都是真實 MTC 規劃）：

| 情境 | 設定 | 結果 |
|---|---|---|
| 迴歸：不傳 candidates_path | 預設值 | `hardcoded_single+place_nominal` 第一次嘗試即 PASS，跟改動前行為一致（attempts=1） |
| 8 個真實 grasp 候選 | `candidates_path` 指向 S3 候選 JSON | `G004+place_nominal` 第一次嘗試即 PASS（attempts=1），跟改動前一致 |
| 刻意在 place 原點放一個碰撞物（新增的 `--debug_block_place_nominal` 測試鉤子） | 步長 1.5cm／3cm | 原點如預期失敗（`eef in collision: debug_place_blocker - gripper_base` 等），但**四個鄰居偏移也全部失敗**——因為夾爪本身開口達 6.9cm，1.5–3cm 的偏移不足以讓夾爪整個閃開一個貼著桌面、2cm 見方的障礙物 |
| 同上，步長放大到 10cm（僅測試用，非預設值） | | 原點 FAIL（`Collision between debug_place_blocker - gripper_base`），**`place_x+` 偏移 PASS**，`attempts=2`，證明迴圈確實會往下嘗試且能找到可行的鄰居 |

第三個情境本身就是一個誠實的發現，不是失敗的測試：它具體示範了上面「範圍限制」講的「這是簡化
版，不是真正的碰撞感知搜尋」是什麼意思——固定 1.5cm 步長的搜尋半徑，面對一個跟真實夾爪開口
（6.9cm）同量級的障礙物是不夠用的，要嘛障礙物本身要夠小、要嘛需要更大的搜尋半徑或更聰明的
搜尋策略（例如沿障礙物邊緣走，而不是固定四方向網格）。第四個情境把步長開大到 10cm 純粹是為了
證明「迴圈邏輯本身沒問題，鄰居確實可以被選中」，10cm 不是建議的正式參數。

**新增的測試鉤子**：`--debug_block_place_nominal`（預設 `false`）在 place 原點附近加一個小型
碰撞物，`buildAndPlan()` 新增一個可選的 `extra_obstacle` 參數用來注入它。這是永久留在程式碼裡
的測試能力（不是用完即丟的一次性 hack），之後要驗證位置修正邏輯還可以直接重用。

**視覺化同步更新**：`buildCandidateMarkers()` 現在同時畫 grasp 與 place 兩組箭頭＋標籤
（namespace 分別是 `grasp_candidates` 與 `place_candidates`），共用同一個 DELETEALL、同一次
publish，上面「候選決策視覺化」一節的操作步驟不用改，RViz 打開後兩種候選都看得到。

**範圍限制（誠實記錄）**：偏移網格固定是「原點＋四鄰居」，不會隨失敗自動擴大或改變密度；只有
xy 平移，沒有嘗試不同的 place 朝向；`max_candidates`（grasp 側）與五個 place 偏移是巢狀相乘，
worst case 到 `max_candidates × 5` 次完整 MTC 規劃，目前的預設值（8）在觀察到的單次規劃時間
（約 0.2–1.5 秒）下還算可接受，但沒有針對更大 `max_candidates` 做效能測試。

## S4：感知整合 — 完成一條真實端到端鏈路，對 live Isaac 驗證

之前每一輪的「還沒做的部分」都寫著同一句話：「還沒接上真正的 VLM 呼叫、還沒接上腕上相機的真實
RGB-D」。這輪把這兩個缺口都補上，跑出一條**全部用真實資料、沒有任何合成/假造步驟**的完整鏈路：

```
live Isaac 腕上相機（真實模擬 RGB-D）
  → capture_scene.py --best-effort（真實 ROS 訂閱擷取）
  → run_grasp_affordance()（真實本地 VLM 呼叫，非假 backend）
  → 顏色分割物體遮罩（真實色彩門檻，非 GT mask）
  → object_points_and_task_region_mask()（真實深度反投影）
  → 真實 T_world_camera（TF 實際查表，非假設值）
  → generate_grasp_candidates()（真實候選生成）
  → robot129_mtc_pick_place（真實 MTC 規劃，對 live Isaac）
```

**新增檔案**：
- `research/scripts/capture_scene.py`：新增 `--best-effort` 旗標（預設關閉，不影響既有行為）。
  發現：Isaac 模擬相機的 sensor QoS 是 BEST_EFFORT，但這支既有腳本原本的相機訂閱寫死
  RELIABLE，兩邊 QoS 不相容時 rclpy 只會印一則 WARN、不會報錯，訂閱端就是靜靜收不到任何訊息，
  最後以「timeout」失敗收場——不看 WARN 訊息很容易誤判成 TF 或同步問題。加旗標修正，不改預設
  行為（真實相機驅動通常本來就是 RELIABLE，原本的預設值保留給那個情境用）。
- `research/scripts/s4_live_affordance_smoke_test.py`：新端到端腳本，串起上面整條鏈路。物體遮罩
  目前是「紅色門檻分割」（cube 的 `diffuse_color=(0.88,0.08,0.04)`），明確標註是佔位符，不是真正
  的分割模型——`affordance_region.py` 的 docstring 跟先前的報告都已經點名這是還沒做的部分，這裡
  用最簡單能動的方式頂上，真正的分割模型仍是未來工作。

**對 live Isaac 的一次真實跑**（`s4_live_wrist_capture_01`，pick_place 場景 HOME 姿態）：

1. **擷取**：`capture_scene.py --best-effort` 從真實的
   `/robot129_sim/camera/color/image_raw`、`aligned_depth_to_color/image_raw`、
   `aligned_depth_to_color/camera_info` 擷取一張同步快照，TF 查到真實的
   `T_world_camera`（camera 在世界座標 (0.345, 0, 0.395)，朝下看，跟腕上相機由上往下看的預期
   一致）。RGB 影像清楚拍到紅色方塊跟綠色放置標記。
2. **VLM 呼叫**：`run_grasp_affordance()` 打真正的本地 vLLM（`tools/start_robot129_vllm.sh`，
   Qwen3-VL-8B-Instruct，自架不花錢，非付費 API），一次呼叫成功（`n_calls=1, parse_retries=0`），
   回傳 `status=LOCALIZED, description="red cube body", bbox_yx_norm1000=(450,450,550,550)`——
   VLM 真的把框畫在方塊上，不是隨便回一個值。
3. **物體遮罩 ∩ VLM 框 → 3D 任務區域**：顏色門檻抓到 7133 個方塊像素，跟 VLM 框交集後剩
   1914 個點；反投影＋世界座標轉換後，**這些點的世界座標範圍是 x∈[0.298,0.338]、
   y∈[−0.019,0.019]、z∈[−0.000,0.035]**——跟已知的真實方塊幾何（中心 (0.32,0,0.0175)，邊長
   0.035，即 x∈[0.3025,0.3375]／y∈[−0.0175,0.0175]／z∈[0,0.035]）幾乎完全吻合（誤差在
   1cm 以內）。這個數字本身就是對整條相機外參＋內參＋TF 鏈路最有力的驗證：任何一段算錯，這個
   比對都不可能這麼準。
4. **候選生成**：8 個候選，6 個 accepted，最高分（最安全）是 `G000`（score=0.1660，明顯優於
   人工合成候選跑出來的 0.5072——因為這次是真實點雲、邊緣分布更自然，不是我手排的立方體格點）。
5. **MTC 規劃**：把 `s4_live_candidates.json` 直接餵給既有的
   `robot129_mtc_pick_place`（沒有改任何規劃程式碼），`G000` 的五個 place 偏移**全部因為
   `no IK found`（真實運動學不可達）而失敗**——這是真實限制，不是 bug（矩合 S0 記錄的 joint5
   ±1.22 rad 限制：`G000` 的抓取高度是候選生成器算出來的 z=0.0266，比先前已驗證成功的
   z=0.0175 高，加上它的特定 yaw 角度，剛好落在不可達的姿態）。迴圈正確往下嘗試，第六個候選
   `G006` 的原點 place 第一次就規劃成功，**`status=PASS`**。

**這個結果證明了什麼**：不是「湊巧成功」，而是每一段都可以獨立驗證——VLM 真的定位對了物體、
幾何轉換真的準確（跟已知 ground truth 誤差 <1cm）、候選生成器面對真實（非人工合成）點雲一樣能
正確運作、MTC 規劃正確拒絕了一個真的不可達的候選、迴圈正確 fallthrough 到下一個並成功。整條
ZeroDex 式管線（真實相機 → 真實 VLM → 真實 3D grounding → 真實候選生成 → 真實動作規劃）第一次
被完整跑過一次，不再是「各段分開測試過，沒串起來看過」。

**誠實記錄：這一次成功不代表整條鏈路已經「可靠」**：
- 只跑過一次，只有一個場景（HOME 姿態看 pick_place 的紅色方塊）、一個物體、一個指令描述。
  沒有重複跑多次看穩定性，也沒有换物體、换場景、换姿態測試。
- 物體遮罩是顏色門檻，換一個不是純色、或背景色接近的物體就會失效——這是刻意先求「跑得通」的
  佔位符，不是可以直接推廣的方法。
- VLM 只被問了一次「這裡能不能抓」，沒有測試「VLM 講錯話」（例如框到背景、框到錯誤物體）時
  下游會發生什麼——`AffordanceRegionError`／`GroundingFailure` 的例外處理路徑目前只在單元測試
  的合成資料上驗證過，這次真實跑剛好沒有觸發它們。
- 沒有測試論文真正關心的「任務條件式」場景（例如「抓瓶身、不要抓瓶蓋」）——這次的指令
  「pick up the red cube」沒有需要區分可抓／不可抓子區域的任務語意，`bbox` 幾乎框住整個物體，
  沒有真正測試到「排除功能端」那個機制在真實資料上是否也成立。

## S5：task-region pilot 實驗 — 完成（GT 幾何離線 pilot，非正式評測）

依計畫書 Sec.9／handoff 計畫的 S5 範圍：同一物體、不同任務區域的鎚形物案例（分「握柄」與
「握頭」兩種任務），跑 B0（單一預設 yaw）／B1（多 yaw，整個物體都可抓）／B2（多 yaw ＋任務區域
過濾）三種基準比較，4 場景小規模 pilot。**這輪用 GT 幾何（不是 VLM／不是真實分割），刻意把
「任務區域過濾有沒有用」跟「VLM／分割找不找得到那個區域」這兩個問題分開驗證**——後者已經在
S4 用真實 VLM 跑過一次（雖然那次的指令沒有需要區分子區域），這裡專門測前者。

**新增檔案**：`research/scripts/s5_pilot_hammer.py`——用跟
`tools/generate_s2_cube_candidates.py` 相同手法（顯式表面點取樣，不用 mesh／USD 資產）建一個
鎚形物：一根細長握柄（0.14×0.02×0.04m）加一個較粗的鎚頭（0.05×0.045×0.05m），物體中心對齊
S2 已驗證過的可達範圍（x=0.32），對 4 個「場景」（握柄任務×2 個 yaw、握頭任務×2 個 yaw）各跑一次
B0／B1／B2。

**接線時抓到的一個真實 bug**：鎚頭比握柄高，第一版兩個部件共用同一個中心 z，導致鎚頭的箱子有一
部分穿到桌面以下（z 最小值到 −0.006m）。這個 bug 沒有直接報錯，而是安靜地讓每個候選都因
`TABLE_CLEARANCE` 被拒絕，全部 0 個 accepted——先印出 `points[:,2].min()` 才發現物體本身的
z 範圍就不對。修正成兩個部件各自的中心 z 都讓自己的底部落在 z=0（握柄中心 z=0.02，鎚頭中心
z=0.025），問題消失。

**Pilot 結果（4 場景，不是正式評測，數字不能當基準引用）**：

| baseline | 正確落在任務區域內 |
|---|---|
| B0（單一預設 yaw，無任務過濾） | 1/4 |
| B1（多 yaw 掃描，整個物體可抓） | 3/4 |
| B2（多 yaw ＋任務區域過濾） | 4/4 |

最有意義的一筆是 `hammer_head_yaw0`（任務：抓鎚頭）：B1 選中的最佳候選（分數最高、開口最省）
其實落在**握柄**上，不是鎚頭——因為整個物體可抓時，握柄的窄截面天生比鎚頭省開口、分數更好，
「分數最優」跟「任務要的部位」是兩回事，B1 選錯了。B2 因為候選空間本來就被限制在鎚頭範圍內，
同一個場景反而 8 個候選全部 accepted 且全部落在正確區域。**這正是 ZeroDex §3.4 任務條件式
抓取區域存在的理由**：沒有任務過濾時，「幾何上最好抓」跟「任務要抓的地方」可能是物體上完全不同
的兩塊，這個 pilot 用一個可重現、可查驗的小案例把這件事具體演示出來，不是只用文字描述。

**誠實記錄（範圍限制）**：
- 只有 4 個場景，不是計畫書講的正式「12 場景、108 次 holdout」評測，數字僅供這次 pilot 內部
  比較，不能當成任何形式的成功率結論。
- 這個離線 pilot 本身的任務區域遮罩是**手動指定**哪些點屬於握柄／鎚頭（GT）——但下面「S5 延伸」
  已經把這條路線跟 S4 的「VLM 出框 → 顏色分割 → 交集」合起來，在 live Isaac 上真的測過一次。

完整程式碼輸出見 `out/grasp_motion/s5_pilot/hammer_pilot_results.json`。既有 164 個 research
單元測試全數通過，本輪新增的 `capture_scene.py --best-effort` 旗標與新腳本未觸發任何既有測試的
回歸（旗標預設關閉，新腳本不在既有測試發現路徑下）。

### S5 延伸：live Isaac 裡的真實雙色物件 ＋ 真實 VLM 區分部件 — 完成，對 live Isaac 驗證

上面的離線 pilot 只在 `research/` 跑合成點雲。這輪把同樣的「一物體兩部件」概念真正搬進
Isaac 場景，讓真實相機看到、真實 VLM 判斷，把 S4（真實 VLM＋真實相機）跟 S5（任務區域區分
不同子部件）合流成同一條鏈路，在一個新物體上跑一次。

**新增場景 `--scene pick_place_hammer`**（`sim/scripts/run_robot129_ros_webrtc.py`）：跟既有
`--scene pick_place` 共用全部既有邏輯（`pick_place = args.scene in ("pick_place",
"pick_place_hammer")`），只有兩處新增，且都用 `hammer_mode` 旗標包住，**不影響 `--scene
pick_place`／`--scene marker` 的既有行為**（已對兩者重新跑過驗證，見下方）：
1. `target_cube`（既有的抓取目標 prim／topic／MTC 碰撞物件名稱完全不變）改用一個細長箱子
   （0.14×0.02×0.03m，卡其色）取代原本的立方體，當作「握柄」。
2. 新增 `/World/HammerHead`：一個**純運動學**（`kinematic_enabled=True`，不吃 PhysX 動力學）
   的灰色箱子，每個 publish tick 用 <code>update_hammer_head()</code> 把它的姿態設成「握柄目前
   的即時姿態 ∘ 固定局部偏移」，視覺上／深度感測上會跟著握柄走，但**不參與 MTC 碰撞規劃或候選
   執行**（誠實記錄，見下方範圍限制）。

**接線時抓到的一個真實 bug**：第一版握柄跟鎚頭的初始生成位置有 1cm 重疊，握柄是動力學物體、
鎚頭是無限質量的運動學物體，兩者一重疊，PhysX 的穿透排除（depenetration）就把握柄用力推開——
`update_hammer_head()` 要等 ROS publish 迴圈開始才會第一次執行，但生成後、迴圈開始前還有
90 步「靜置」物理模擬，握柄在這段時間被彈飛到世界座標 x=−1.07（離生成點 x=0.32 超過 1 公尺）。
修正成生成時兩個部件之間留出安全間距（超過兩者半寬總和）才解決。

**對 live Isaac 的真實跑**：
1. **場景確認**：`--scene pick_place_hammer` 啟動、擷取真實腕上相機影像，畫面裡是卡其色細長
   握柄＋灰色鎚頭兩個視覺上清楚分開的真實物件（不是合成圖）。
2. **VLM 語意辨識的真實邊界**：直接問 VLM `grasp_object="hammer"` 兩次（分別要握柄／要鎚頭）都
   回傳 <code>UNAVAILABLE, reason_codes=["TARGET_NOT_VISIBLE"]</code>——這是一個**誠實、真實的
   發現**：兩個純色、無材質貼圖、由正上方拍攝的長方體，在這個本地 VLM 眼裡不構成「一把鎚子」的
   語意，跟真實鎚子的輪廓（圓柱握柄、有爪或平面的鎚頭）差太多。改用<b>純視覺描述</b>
   （<code>grasp_object="the beige rectangular bar"</code> / <code>"the gray rectangular
   block"</code>）後，VLM 兩次都正確 <code>LOCALIZED</code>，且框住的是各自對應的真實部件，
   沒有搞混。
3. **完整鏈路（握柄任務）**：VLM 出框 → 卡其色顏色門檻分割（15172 像素）→ 交集反投影 → 世界座標
   <b>x∈[0.250,0.390]、y∈[−0.0095,0.0102]</b>——跟握柄真實幾何（中心 0.32、半長 0.07、半寬
   0.01，即 x∈[0.25,0.39]／y∈[−0.01,0.01]）幾乎完全吻合 → 8 個候選、6 個 accepted，最佳候選
   <code>G004</code>（score=0.2845）落在握柄中心 (0.3366, 0.0004, 0.03)。

這證明了兩件事：**(a)** S4 那條「真實相機→真實 VLM→真實 3D grounding→真實候選生成」的鏈路不是
只對 S2 的立方體有效，換一個真正不同形狀、不同顏色配置的物件，整條鏈路照樣正確跑通、且反投影出
的世界座標依然跟已知真實幾何吻合在 1cm 以內；**(b)** 「VLM 能不能認出這是什麼」跟「VLM 能不能
照描述框出正確區域」是兩件事——這個簡化的方塊組合物件在語意辨識上失敗了，但在視覺描述辨識上
完全成功，這個邊界本身就是有價值的誠實記錄，不是要美化掉的缺陷。

**範圍限制（誠實記錄）**：鎚頭是運動學物件，這輪**沒有**把候選送進 MTC 對 live Isaac 執行
抓取——MTC C++ 節點的碰撞防護物件（`target_cube`，寫死 0.035m 立方體）大小跟握柄的真實外形
（0.14×0.02×0.03m）不符，要讓 MTC 規劃時的碰撞防護跟真實物件形狀一致，需要把 C++ 那個寫死尺寸
改成可設參數（跟這輪其他新增的 `place_offset_step_m` 等參數同樣的模式），這輪為了不去動一個已經
被大量驗證過的核心規劃檔案而選擇不做；「VLM／幾何/候選生成對新形狀依然正確」已經是這次要驗證
的核心問題，MTC 執行留給下一輪。物體語意識別（「這是鎚子」）本身也還沒解決——這輪繞過去用視覺
描述取代，不是解法，是誠實記錄的已知邊界。

## 修改與新增檔案清單

新增：
- `tools/analyze_top_down_reach.py`
- `tools/start_robot129_grasp_sim.sh`、`stop_robot129_grasp_sim.sh`、`status_robot129_grasp_sim.sh`
- `tools/verify_grasp_motion_manual_waypoints.py`、`.sh`
- `ros2_ws/src/robot129_sim_execution/`（整個新 ament_python 套件：adapter node、robot model、
  `run_grasp_motion.py` 執行器、launch、config、pytest 測試）
- `ros2_ws/src/robot129_tasks/src/robot129_mtc_pick_place.cpp`（真正的 MTC pick-place 任務）
- `ros2_ws/src/robot129_tasks/launch/mtc_pick_place_sim.launch.py`
- `ros2_ws/src/robot129_moveit_config/launch/move_group_sim.launch.py`（`/robot129_sim`
  namespace 版本，供 MTC 的 `CurrentState` 使用；原本根 namespace 的
  `move_group.launch.py` 保留給既有 `verify_moveit_fixed_plan.sh` regression 用）
- `research/src/mpg/grasp_candidates.py`、`grasp_contract.py`（S3 幾何候選生成器＋JSON
  contract）、`research/configs/grasp_generation.yaml`、`research/tests/test_grasp_candidates.py`
- `docs/progress/grasp_motion_s0_inventory.md`、`grasp_motion_interface_map.yaml`、本檔

修改：
- `sim/scripts/run_robot129_ros_webrtc.py`（見上；`--scene marker` 預設行為不變）
- `ros2_ws/src/robot129_moveit_config/config/robot129.srdf`（`collisions_updater` 重產
  完整碰撞矩陣；`end_effector` 補 `parent_group="arm"`；`open` group_state 從 0.035/-0.035
  改成 0.0345/-0.0345，見上方除錯記錄第 5 點的邊界問題）
- `ros2_ws/src/robot129_moveit_config/config/moveit_controllers.yaml`（補
  `moveit_controller_manager` key）
- `ros2_ws/src/robot129_moveit_config/config/kinematics.yaml`（KDL timeout 0.1→0.3）
- `ros2_ws/src/robot129_tasks/CMakeLists.txt`（新增 `robot129_mtc_pick_place` 執行檔；補
  `link_directories($ENV{CONDA_PREFIX}/lib)`，只影響本套件）

未修改：`robot/vendor/`、URDF/USD、既有 lesson 報告與驗收 artifact；根 namespace 的
`move_group.launch.py` 與 `robot129_mtc_plan.cpp`（舊 demo）也未變動，`verify_moveit_fixed_plan.sh`
regression 重跑過仍 PASS（22 點、終點誤差 0.0009 rad，`moveit_controller_manager` 缺失的
WARN 也消失了）。

## 下一步（2026-09-20 盤點與修正後更新，已完成的四項移除）

1. 物體分割目前是兩支腳本裡各自手刻的顏色門檻（不同參數），沒有共用模組、沒有真正的分割
   模型。
2. （選用）匯出時接上 MTC `Introspection`，讓 `sub_trajectory` 的 comment 直接是 stage
   名稱，不用再靠關節組成或位置猜測。
3. 正式 12 場景、108 次 holdout 評測（S5 目前只有 4 場景 pilot）。
4. MTC 的 `target_cube` 碰撞防護目前仍是立方體形狀（`scene_geometry.json` 只提供尺寸數字，
   C++ 節點還沒有真正依物體形狀改變碰撞 primitive）；鎚形物件的候選還沒有送進 MTC 實際執行過。
5. `derive_grasp_hint()` 目前只處理 pick-place 的 GRASP 步驟，工具類任務的
   FUNCTIONAL_TIP／APPLY_ACTION 步驟還沒有對應的 affordance 條件化邏輯。

## 如何自己重現

**建議：一行指令跑完整條鏈路**（`tools/run_grasp_motion_demo.sh`，2026-09-20 新增，取代
下面手動 3 步驟＋2 個 Python 環境的流程）：

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
bash tools/start_robot129_grasp_sim.sh --scene pick_place --seed 7   # 先啟動場景
bash tools/run_grasp_motion_demo.sh --record                        # reset→候選→規劃→真實執行
# 結果在 out/grasp_motion/mtc_execution/<timestamp>_demo/report.json
```

以下是該腳本內部串起來的手動版本，供除錯或想個別檢查每一步時使用：

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913

# 1) 啟動 headless pick_place 場景（不占用 WebRTC port 49100）
bash tools/start_robot129_grasp_sim.sh --scene pick_place --seed 7

# 2)（可選）先用手動 waypoint 驗證控制鏈本身沒問題
bash tools/verify_grasp_motion_manual_waypoints.sh --record

# 3) 建置 MTC 與 adapter 套件（需要 CONDA_PREFIX/lib 在 LD_LIBRARY_PATH，見上方除錯記錄 1–2 點）
cd ros2_ws && colcon build --packages-select robot129_moveit_config robot129_tasks robot129_sim_execution
source install/setup.bash

# 3.5) 規劃前先 reset 場景（2026-09-20 才發現這個順序很重要——見下方修訂記錄第 5 點：
#      如果場景是上一輪執行完、手臂在 place 姿態、方塊已經搬走的狀態，規劃出來的軌跡
#      第一個點會跟「執行器自己也會做的 reset」之後的真實起始狀態對不上，第一段就被拒絕）
ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ros2 service call /robot129_sim/reset_scene std_srvs/srv/Trigger

# 4) 規劃：MTC 對 live Isaac 狀態規劃取放任務（同時會起一個 plan-only move_group）
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ros2 launch robot129_tasks mtc_pick_place_sim.launch.py
# 結果在 out/grasp_motion/mtc_pick_place.json，status 應為 PASS

# 5) 另開一個 shell：啟動 adapter（對同一個 live Isaac）
ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run robot129_sim_execution adapter_node \
  --ros-args --params-file src/robot129_sim_execution/config/adapter_params.yaml \
  -p log_dir:=/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/out/grasp_motion/adapter \
  -r __ns:=/robot129_sim

# 6) 執行 MTC 規劃出的取放任務並錄影
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ros2 run robot129_sim_execution run_grasp_motion --plan out/grasp_motion/mtc_pick_place.json --record

# 7) adapter 的獨立單元測試（不需要 Isaac/GPU）
cd ros2_ws/src/robot129_sim_execution/test
python3 -m pytest -v -p no:launch_testing -p no:launch_ros .

# 8) 用完記得關閉
bash tools/stop_robot129_grasp_sim.sh
```

若要用 WebRTC 即時看（而非只看錄影），需要你先關掉現有 parcel-forge viewer（占用同一組
固定 port 49100/47998），再改用 `tools/start_robot129_ros_webrtc.sh --scene pick_place`
（不加 `--record-only`）。這是你自己的其他工作，我不會替你關閉。

## 2026-09-20 修訂：盤點後修正的真實 bug，與「規劃 PASS」到「真的執行成功」的落差

9/18 之後新增的東西（多候選迴圈、place 偏移搜尋、決策視覺化、S4 真實 VLM、S5 鎚形物）都只
驗證到「MTC 規劃回報 PASS」為止，**最後一次真正把規劃結果送進 adapter 執行、看著方塊真的被
夾起來放下，是 9/18 21:54**。這輪先做了一次自我盤點（見下方「盤點結果」），找到並修正六個
真實問題，其中三個是靜態程式碼審查就能發現的，另外三個是**只有真的重新執行過才會浮現**的：

**A. 靜態盤點發現的問題**：
1. **鎚形物尺寸在兩個檔案裡各自寫死，早已不一致**：Isaac 實際生成的握柄高 0.030m、鎚頭
   0.045m，但 `research/scripts/s5_pilot_hammer.py` 離線算候選用的是 0.040m／0.050m——S5
   pilot 的 B0/B1/B2 結果其實是算給一個 Isaac 裡不存在的物體。修正：新增
   `research/configs/scene_geometry.json` 當單一真實來源，`sim/scripts/
   run_robot129_ros_webrtc.py`、`research/scripts/s5_pilot_hammer.py`、
   `tools/generate_s2_cube_candidates.py`、`robot129_mtc_pick_place.cpp`（用既有的
   `tinyjson` parser讀取，失敗時退回寫死值並印 WARN，不會讓節點啟動失敗）都改成讀這個檔案。
2. **執行器的 `REST_Z = 0.0175` 寫死方塊半高**，換成鎚形物握柄（半高 0.015）會在
   `place_error_z_m` 產生固定 2.5mm 偏差。改成 `--rest-z-m` 參數，預設從
   `scene_geometry.json` 讀 `cube.rest_z_m`。
3. **`reset_scene` 不會重設鎚形物的頭部**（純運動學物件，只在每個 ROS publish tick 由
   `update_hammer_head()` 從握柄姿態推算）；**執行器完全忽略 reset service 回傳的 `success`
   欄位**，reset 被 busy guard 擋掉時會靜靜地在沒重設的場景上繼續跑。兩者都已修正：reset
   handler 內重設握柄後立刻呼叫一次 `update_hammer_head()`；執行器現在檢查 `success`，
   False 就直接 FAIL 退出，不會假裝場景已重設。

**B. 只有真的執行過才會發現的問題**（這是這輪最重要的部分——重新驗證「規劃 PASS」到「真的
執行成功」這條路徑，過程中連續發現三個問題）：

4. **adapter 的 arm 關節容忍值 0.02 rad 對 RRTConnect 的實際精度來說太緊**：兩次獨立的真實
   執行（不同 Isaac 行程、每次都重新規劃）都在「connect to place」這段 RRTConnect 移動的
   joint6 上，以幾乎相同的誤差量（0.0244 rad、0.0225 rad）卡在容忍值外——其餘關節始終在
   0.009 rad 以內。這是可重現的系統性精度極限，不是偶發或這輪其他改動造成的。修正：
   `arm_goal_tolerance_rad` 從 0.02 調到 0.03（比兩次實測的較差值留約 25% 餘裕），
   `config/adapter_params.yaml` 與 `adapter_node.py` 的預設值都改了，並在檔案裡完整記錄
   這個決定的實測依據。
5. **`tools/run_grasp_motion_demo.sh`（新增的一行指令包裝腳本）第一版重跑會失敗**：MTC
   規劃是根據「上一輪執行完、方塊已經被搬到 place 位置」的即時場景狀態去算（MTC 的
   `CurrentState` 讀真實 joint_states，但候選姿態本身、還有執行器自己在執行前的
   `reset_scene` 呼叫，都假設方塊在 nominal 位置），執行器一啟動就把手臂 reset 回 HOME，
   跟剛剛規劃時的假設狀態不符，adapter 直接拒絕第一段軌跡。修正：包裝腳本改成**規劃前先
   reset**，讓執行器自己的 reset 變成一個無操作，而不是規劃完之後才第一次改變場景狀態。
6. **包裝腳本殺 adapter 子行程時只殺到 `micromamba run` 這層 wrapper，沒殺到真正的
   `adapter_node`**：因為 `micromamba run` 不會 exec-replace 自己，真正的 ROS 節點是孫行程，
   父行程被殺掉後它變成孤兒繼續跑。下一次重跑時，殘留的舊 adapter 跟新啟動的 adapter 同時
   註冊成同一個 action 的伺服器，goal 路由變得不確定（`ros2run` 印出
   "Ignoring unexpected goal response... more than one action server"），新的執行請求被
   靜靜拒絕。修正：清理函式比照 `tools/stop_robot129_grasp_sim.sh` 已有的做法，直接殺
   `kill` 之外再補一次 `pkill -f`，並在啟動後用 `pgrep` 確認、而不是只看 `micromamba`
   wrapper 那層的 PID 還活著沒。
7. **意外發現，順手修的第七個問題**：Isaac runner 的錄影 `run_id` 計數器每次進程重啟都從
   -1 開始（第一次錄影變成 `run_0000`），這次連續重啟 Isaac 測試時，兩次新錄影**真的覆蓋掉
   了 9/18 的 `run_0000`／`run_0001` 舊影片**（新內容是今天的真實成功執行，換掉的舊檔案已經
   無法復原）。修正：啟動時掃描 `sessions/run_*` 現有目錄，從最大值接續編號，不再每次都從
   0 開始。

**這輪的完整驗證流程**：修好 A 的三個問題後，對 live Isaac 重新跑過一次 `mtc_pick_place_sim.
launch.py`（沒有 WARN 代表 `scene_geometry.json` 讀取成功、跟改動前候選選擇結果一致，無
回歸）；接著實際執行（B 的三個問題就是在這個過程中依序踩到、依序修好的），最終
`tools/run_grasp_motion_demo.sh --record` 完整跑通一次：`status=PASS`，
`trajectory_success`／`physical_grasp_success`／`task_success` 全部 `true`，place
z 誤差 1.06×10⁻⁷m。adapter 的 9 個既有單元測試在改動後重跑仍全數通過。

### D. stage_b 的語意點接上 affordance 呼叫 — 完成，對真實 VLM 驗證

盤點發現的研究深度缺口：`run_stage_a`/`run_stage_b` 定位出的語意點（GRASP／WAYPOINT／
RELEASE 等）跟 `run_grasp_affordance` 之間完全沒有接起來——後者的 `grasp_object`／
`grasp_hint` 一直是呼叫端手寫的字串（例如 `s4_live_affordance_smoke_test.py` 原本寫死
`"red cube"` / `"pick up the red cube"`），跟 stage_b 實際找到了什麼完全無關。ZeroDex
式 9 的 `l‴`（affordance prompt）在論文裡明講是「以前一步找到的抓取關鍵點描述為條件」組
出來的，我們的實作沒有做到這件事。

**新增**：`research/src/mpg/grounding.py` 的 `derive_grasp_hint(located_steps, *,
instruction)`——在 `located_steps` 裡找 `StepType.GRASP` 那一步，用它的 `desc`
當 `grasp_object`、`geometric_meaning` 接在 `instruction` 後面組成 `grasp_hint`；找不到
GRASP 步驟或該步驟沒有 `LOCALIZED`（例如物體判斷不可見）就拋 `NoGraspStepError`，不會
安靜地退回猜一個。3 個新單元測試（`tests/test_grounding.py`
`DeriveGraspHintTest`）：正常情況正確萃取、無 GRASP 步驟時拋錯、GRASP 步驟
`UNAVAILABLE` 時拋錯。

**接上真實呼叫端驗證**：`research/scripts/s4_live_affordance_smoke_test.py` 改成先跑
`run_stage_a`→`run_stage_b`→`derive_grasp_hint`，再把結果餵給 `run_grasp_affordance`
（原本只單獨呼叫 `run_grasp_affordance`），對 live Isaac 擷取的真實紅色方塊影像跑一次：

```
stage_a: status=READY mode=PICK target=red cube (partial)
stage_b: GRASP (498,550) WAYPOINT (850,750) RELEASE (950,750) 全部 localized
derived: grasp_object='red cube'
         grasp_hint='pick up the red cube and place it on the green pad -- grasp
                     target: The top face of the red cube, centered on its visible
                     portion, which is a square with a reddish-orange color and a
                     slight shadow beneath it.'
affordance: LOCALIZED, bbox 跟之前手寫字串版本的結果一致
```

過程中還抓到一個真實的指令設計問題：原本測試腳本的指令是 `"pick up the red cube"`（沒講
放哪裡），`run_stage_a` 正確判斷這樣沒辦法回報 `status=ready`（schema 規則：pick-place
模式的 `ready` 狀態要求有一個可見或部分可見的 `destination`），因為指令沒講目的地、VLM 也
沒有自己發明一個——這是 schema 驗證正確在運作，不是 bug。改成
`"pick up the red cube and place it on the green pad"`（跟 S5 延伸那次鎚形物測試用的
「grasp handle, push the pad」句型一致）後，`stage_a` 正確回報 `ready`。

**誠實記錄**：目前呼叫鏈是 stage_a → stage_b → affordance 依序三次獨立 VLM 呼叫（之前只有
affordance 一次），沒有把三者合併成論文附錄 D.2 提到的單一結構化呼叫；`derive_grasp_hint`
只處理 GRASP 步驟，沒有處理工具類任務的 FUNCTIONAL_TIP／APPLY_ACTION 步驟該怎麼接（那些
步驟本來就標記 `unsupported_by_executor`，這輪沒有擴充範圍）。

**這輪盤點但沒有修的部分**：
- 物體分割仍是兩份獨立的顏色門檻程式碼，沒有共用模組。
- `src/mpg/candidates.py`（舊的 2D 網格點選方案，跟新的 `grasp_candidates.py` 只是名字像、
  功能完全不同）還留著，因為 `run_grounding.py --mode candidates` 等三個呼叫點還在用；沒有
  在這輪清掉。
- MTC 的碰撞防護物件仍是立方體形狀，鎚形物候選還沒有送進 MTC 實際執行過（前一輪就記錄過
  的範圍限制，這輪沒有進一步處理）。

既有 164 個 research 單元測試 ＋ 這輪新增的 3 個，現在是 **167 個全數通過**。
