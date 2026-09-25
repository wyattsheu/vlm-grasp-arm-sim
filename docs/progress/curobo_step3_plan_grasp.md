# cuRobo Step 3 — `plan_grasp.py`，接既有執行鏈，真的跑通 pick-and-place

狀態：PASS（用真實 Isaac 跑通完整 pick-and-place，且和 MTC 做過 A/B 對照）；大樣本可靠度統計 NOT RUN

完整指令、規劃流程、已知簡化、A/B 數字見 `docs/dev_guide_paper_core_and_dashboard_plan.md` §7.6。

## 概念與架構

`research/src/mpg/curobo_bridge/plan_grasp.py` 讀 `grasp_candidate_v0` 候選 JSON，用 cuRobo 的 `MotionPlanner.plan_grasp()`（approach→grasp→lift 一次呼叫）加兩次 `plan_pose()`（transport、lower）規劃完整 pick-and-place，輸出跟 MTC 完全同格式的 `sub_trajectories` JSON，交給既有、沒改過的 `run_grasp_motion.py` / adapter 執行。`tools/run_curobo_grasp_demo.sh` 是一鍵版本，結構對應既有的 `tools/run_grasp_motion_demo.sh`。

## Done／Verified

- 修好一個安裝遺留問題：`robot129.yml` 的 `load_collision_spheres`／`num_envs` 欄位跟 loader 自己的同名參數衝突，`tool_frames` 混了一個沒給目標姿態的相機 frame 導致 IK 求解器出錯——都修掉了（詳見 dev guide）。
- **真的用 Isaac 跑通完整 pick-and-place，不只是規劃成功**：手動跑一次、用 `tools/run_curobo_grasp_demo.sh` 跑兩次，其中 2 次（1 手動＋1 腳本）`task_success=true`：方塊從 `(0.32,0,0.0175)` 被真的夾起（link7/link8 接觸力 ~10N）、抬高 ~5cm、搬到 `(0.27,-0.12)` 附近（誤差 < 0.5mm）、放下、鬆開。
- 跟 MTC 用**同一組候選**做過一次 A/B：兩邊都 PASS，物理結果相近（cuRobo 放置誤差 0.4mm、MTC 2.2mm；lift 高度 cuRobo 5.0cm、MTC 6.0cm）；cuRobo 單獨規劃時間 6.9–17s，MTC 整條 pipeline wall-clock 2m33s（沒能單獨量到 MTC 純規劃時間）。
- 一鍵腳本 `tools/run_curobo_grasp_demo.sh` 本身也真的跑過（不只是寫完沒測），reset→候選→規劃→adapter→執行全部串起來能動。

## NOT RUN／UNVERIFIED

- 4 次完整流程裡有 2 次在 `FollowJointTrajectory` 的容忍值檢查卡住（1 次是夾爪開合太快、已經調長時間修掉；1 次是手臂到位時剛好超過 0.03 rad 容忍值，重跑同一個 plan 就過了）——判斷是 GPU 被佔用造成的即時速度波動，不是 cuRobo 規劃本身的問題，但只有 4 次樣本，沒有量化實際發生率。
- lift 時沒有把目標物當 attached object 加進 cuRobo 的碰撞世界（跟 MTC 不一樣），對雜亂場景是真正的缺口，目前只對單一物體、空曠場景驗證過。
- 只測過 `pick_place` 場景（已知方塊幾何）；`pick_place_counter`、`pick_place_hammer`、S4 live-VLM 候選都還沒拿去跑 cuRobo。
- `--include-pole` 選項沒有實際搭配 `--scene-camera pole` 一起跑過（Step 2 的相機和 Step 3 的規劃器目前是分開驗證的，還沒同一次跑）。
