# cuRobo Step 1 — robot129 的機器人設定與座標橋接

狀態：PASS（`robot129.yml` 建好、FK 交叉驗證、`frames.py` + 9 個單元測試全過）

完整指令、逐欄位的修改內容、坑點見 `docs/dev_guide_paper_core_and_dashboard_plan.md` §7.1/§7.2 — 這裡只記結論。

## 概念與架構

用 cuRobo 的 `RobotBuilder`（`build_robot_model` 工具）從 `ros2_ws/src/robot129_description/urdf/robot129.urdf` 擬合碰撞球，輸出 `research/configs/curobo/robot129.yml`；手動補上 `pinch_center`（真正的 TCP，gripper_base + 0.125m）、`tool_frames`、SRDF home 當 `default_joint_position`、鎖 `joint7`（夾爪開合跟 MTC 一樣分開規劃）。另外寫 `research/src/mpg/curobo_bridge/frames.py` 處理 `grasp_candidate_v0`（xyzw）↔ cuRobo `Pose`（wxyz）跟 GraspGen-X 慣例（closing axis +X）↔ robot129 真實閉合軸（驗證過是 local Y，不是 X）的轉換。

## Done／Verified

- `robot129.yml`：58→68 顆碰撞球，10 個 link 都有（`link6` 是 4mm 薄法蘭盤，擬合品質差是已知限制，見 dev guide 表格，判斷不影響實際碰撞偵測，先不處理）。
- `pinch_center` FK：cuRobo `Kinematics.compute_kinematics()` 跟獨立用 `yourdfpy` 對同一份 URDF 算的結果，位置誤差 < 0.001mm，四元數方向一致（差一個雙覆蓋正負號）。遠優於 1mm 的驗收標準。
- 發現並記錄一個既有疑似 bug：`grasp_candidates.py` 把閉合軸評分定義在 local X，但機械手指實際沿 local Y 分開（兩次獨立用 `yourdfpy` 對 URDF 數值驗證過）。屬於 S3 範圍不是這次的範圍，記在 `frames.py` docstring 和 dev guide 裡，沒有動 `grasp_candidates.py`。
- `research/src/mpg/curobo_bridge/frames.py` + `research/tests/test_curobo_frames.py`：9 個 test case，兩個 venv（`env_robot129_research`／`env_robot129_curobo`）都跑過，全過（`env_robot129_research` 底下 2 個牽涉 `yourdfpy` 的 case 是刻意 skip，不是失敗）。
- 既有 182 個（167 + 新增後）research 測試在 `env_robot129_research` 底下重跑過，全過，確認新增的 `curobo_bridge` 套件沒有弄壞任何既有東西。

## NOT RUN／UNVERIFIED

- 手腕相機的碰撞球沒有做——`AGENTS.md` 規定不可亂猜沒量測過的實體尺寸，這裡照規定跳過，等有量測數據再補。
- 還沒拿既有的 S4/S5 抓取候選跑過 cuRobo IK 跟 MTC/KDL 結果對照（計畫 Step 1 驗證項目之一），排在下一步 `plan_grasp.py` 做的時候一起做。
