# 待確認事項與規格差異

更新：2026-09-11 21:10，Phase 1 gate。隔離 ROS 環境已建立。

## Phase 1 gate — 狀態

1. ~~是否核准對 `1-2.4` 執行 software unbind/bind 與 hub port disable/enable？~~
   **已核准並執行（21:06、21:22）——相機已恢復。**
   `rs-enumerate-devices` 正常列出 `825312070761`。sysfs 的 `serial` 仍是空的，
   但那是 ASIC serial 且 librealsense 不看它，屬於誤判。詳見
   `docs/camera_serial_root_cause.md`。
2. ~~裝置恢復但既有 process 沒恢復，怎麼處理？~~ **已處理，相機完全恢復。**
   Wyatt 授權後：終止孤兒 PID 1645（只動這一個 process）、重啟 camera pane 1
   （指令不變）、發現 `mm_container` 的 `/dev` 是開機當下的靜態快照缺少
   `video12-17`，Wyatt 再授權 `mknod` 補齊 6 個節點。約 15 秒後
   `RealSense Node Is Up!`，`/stats` 持續 `fps=15.0 stale=false`。完整記錄見
   `docs/camera_serial_root_cause.md`。
3. ~~相機恢復後，是否核准只讀 depth metadata 做驗證？~~ **已核准並完成**：
   depth scale = `0.001` m per raw unit。
4. ~~等照明與 scene 布置完成後，是否核准第一次 subscriber-only S1_01
   capture？~~ **已預先核准**，Wyatt 說布置好就可以直接跑。相機硬體已不是
   阻礙，現在只剩現場照明與 target/destination 布置完成這一個前提。

## 需由系統 owner 確認

3. `robotic_agent_system` 與 `robotic_system` 兩個 decision container，哪一個才接收 demo command？
4. live arm build 是否能提供 build commit／artifact hash，以排除 mount source 與 installed package drift？
5. hand-eye calibration 的來源、日期、誤差、camera optical frame、base frame與 TCP frame 是什麼？
   **2026-09-12 升級為 action generation 阻擋項**：沒有 TCP frame 的定義，
   「指定抓取姿態」這句話沒有明確意義（姿態相對誰而言無法確定）。見
   `docs/action_generation_survey.md` Sec 5。
6. Tuan 的 TCP-to-object grasp transform、orientation policy、joint constraints及離線 path-check 介面為何？
   **2026-09-12 升級為 action generation 阻擋項**：同上，這是「路徑 A：讓 IK
   接受 6D 目標姿態」開工前必須先問到的輸入。
12. 相機支架的幾何尺寸與安裝位置為何？**2026-09-12 新增**：現行 URDF
    （`piper_description.urdf`）只有 base_link–link8 共 10 個 link，
    **不含相機支架或任何環境幾何**，所以沒有任何機制阻止手臂轉進會撞到
    支架的區域，目前靠「指令不要下太大」迴避。這與 action generation 直接
    衝突——姿態取樣與軌跡最佳化都會主動探索該區域。Phu 會中已承認
    workspace 定義是漏掉的基礎工作（「we missed that stuff」）。
13. cuRobo 要在 Jetson 實機安裝，還是先在 I3 的 RTX PRO 6000 驗證？
    **2026-09-12 新增**：ZeroDex 的運動生成元件 [54] 就是 cuRobo，與夾爪
    型態無關、吃 URDF（我們有）、aarch64 自 v0.6.1 起支援。但主機與所有
    執行中容器皆未安裝（已實測 import 失敗），且 Jetson 上的 CUDA/PyTorch
    相依有風險。建議先在非實機環境驗證。
14. GraspGen-X 官方 `piper_hand` asset 是否與 Robot 129 的實際 Piper
    平行夾爪完全一致（URDF、finger travel、approach/closing axes、TCP）？
    **2026-09-12 新增，action-generation feasibility spike 阻擋項**：官方
    repo 的 supported-gripper 清單確實有 `piper_hand`，但名稱相同不能代替
    幾何與 frame 逐項核對。未核對前只能列為候選，不能聲稱可直接部署。
15. 現行 controller 的 orientation tolerance 與完成條件要如何定義？
    **2026-09-12 新增，full-pose IK 阻擋項**：`move_arm_to_pose` 雖以 quaternion
    建立 6D target 並由 `p_servo` 處理姿態，但目前 success criterion 只計算
    平移誤差（`base_action.py:95-102`）。只把 `find_reachable_pose` 改為 full-pose
    target 仍可能在姿態未收斂時提早回報成功。
7. ~~Gemini key 將以哪個變數提供？~~ **2026-09-12 已解決**：Wyatt 提供
   `GOOGLE_API_KEY`（跟 live 系統同一個變數名，不是 `GEMINI_API_KEY`），寫在
   `WORK_DIR/.env`（已加入 .gitignore，未 commit）。過程中發現使用者第一次
   貼上的字串多了一個字元（`A/Iza...` 應為 `AIza...`），已由 Claude 修正並
   用 `models.list` 驗證有效，可存取 `gemini-robotics-er-2-preview`。目前
   `src/mpg/vlm/gemini.py` 預設讀 `GEMINI_API_KEY`；實際呼叫時需另外指定
   `api_key_env="GOOGLE_API_KEY"` 或改預設值，兩個系統統一用哪個變數名
   仍待決定。
8. ~~Molmo2 port 8002 由誰、用哪個啟動規格提供？~~ **2026-09-12 已解決**：
   容器 `local_pipeline_stage2_molmo2`（`allenai/Molmo2-4B`）本來就存在，
   是被人手動 `docker stop` 的（非 crash），Wyatt 核准後執行
   `docker start` 重新啟動，170 秒後 health 正常。**這代表在此之前，
   Robot 129 實機的 `VLM_BACKEND=local` 抓取流程一直是壞的**——
   `mm_actions_node.py` 需要 :8000 與 :8002 都在，Stage 2 定位原本連不上。
   新增 `src/mpg/vlm/molmo.py` 作為正式 backend。

9. ~~論文有沒有公開程式碼？我們該怎麼照著實作？~~ **2026-09-12 已查證**：
   `jlogkim.github.io/zerodex`（project page）標示 "Code (Coming Soon)"，
   目前**沒有**公開 repo。使用者記得的「論文最後面的代碼」其實是
   **Appendix D.2 的 6 個完整 prompt 全文**（Long-horizon Planner /
   Multi-view Roles and Plan Selector / Point Localization / Per-view
   Point Localization / Ray Voting / Grasp Affordance），已在本地
   `refs/zerodex_2606.19340v2.txt:806-1085`。本輪據此新增
   `prompts/stage_0_planner.txt`，並把 tool-use plan 從 3 步改成論文
   PART B 的固定 4 步（補回 FUNCTIONAL_TIP）。
10. 論文用 `gemini-robotics-er-1.6-preview`
    （`refs/zerodex_2606.19340v2.txt:803`），我們手上驗證過的是
    `gemini-robotics-er-2-preview`。本輪消融測試依 Wyatt 指示只測 ER-2，
    **未測 ER-1.6**——若要嚴格對照論文設定，之後需另外確認 ER-1.6 是否
    在目前的 key/專案下可存取。
11. Stage 0 補上、tool-use 改 4 步之後，跑了完整消融
    （3 設定 × 3 replicate × 2 backend，見 `docs/prompt_strategy.md`
    Sec 5.5、`out/report_test/v2/all_replicates_{local,gemini}.json`）。
    機制（4 步 schema + FUNCTIONAL_TIP 定位）已驗證可行：指令明確要求
    用刷子時，Gemini 4 點全對且與論文參考圖吻合，Qwen+Molmo2 4 步都出現
    且點都落在正確物體上。仍未解決、與 schema 無關的兩個語意缺口：
    (a) Qwen 的顏色配對不穩定——只有搭配 Stage 0 才會選對 orange，直接
    單階段判斷會選錯 green；Gemini 反而在三種設定下都穩定選對，不受
    Stage 0 影響；
    (b) 工具判斷不會可靠跟隨指令要求——兩個 backend 的 Stage 0
    都會把「用工具」的 GOAL 分解成較簡單的 PICK+PLACE、忽略工具要求。
    這是 Stage 0 任務分解本身的推理限制，需要另外設計消融或換更強的
    Stage 0 措辭來驗證，不在這次的 schema/prompt 結構修正範圍內。

## 已由證據解決

| 舊假設 | Phase 0 結果 | 後續處理 |
|---|---|---|
| standalone client 是 deployed code | live mount 指向 `robotic_agent/mm_system`，client hash 不同 | baseline 鎖定 live-mounted hash `63588c…` |
| local 用 Ollama | client 是 vLLM endpoints；Ollama models不在該路徑 | 不自行改 backend |
| Qwen+Molmo 每次固定兩階段 | deterministic TaskRequest 先解 action/label；Qwen只是 fallback | 正常 latency/call count按 Molmo單次計，fallback另計 |
| local baseline 可立即跑 | Qwen 8000健康，Molmo預設8002沒有 listener | 標記 unavailable，不能假綠 |
| camera 正常 | viewer stats為 stale；topic取不到新 frame | 人員恢復後才 capture |
| decision repo 唯一 | 兩個 decision container 同時 running | owner 指定 active routing |
| current depth median | live source仍是 11×11 mean | mean baseline；median作單變量消融 |
| normalized ×W/H 安全 | 1000 endpoint可越界 | 新 adapter用 W−1/H−1並保留 legacy reproduction |
| 永不輸出 null | 會把不可見物變猜點 | typed abstention |
| release 加物體高度即可 | point/TCP reference與容器幾何未定 | schema保留 unknown，Phase 3才推導 |
| command-width 等於 grasp成功 | runtime停用 force feedback | 不用現行狀態作 grasp success ground truth |
