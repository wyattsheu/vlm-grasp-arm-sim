# Grasp/Motion S0 — 現況盤點與可達性分析

日期：2026-09-18（Asia/Taipei）。狀態：**唯讀盤點＋離線 FK 計算，未啟動 Isaac／ROS**。
對應交接包 `grasp_motion_research_plan_20260918.md` §7 S0 與 runbook §4。
Workspace（下稱 W）：`/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913`。

## 1. 控制鏈：MoveIt → FollowJointTrajectory → live Isaac

| 欄位 | 現況 | 證據 |
|---|---|---|
| live runner | `sim/scripts/run_robot129_ros_webrtc.py`，node `isaac_articulation_bridge`，namespace `/robot129_sim`，需要 `ROS_DOMAIN_ID=129` | `:132, :243` |
| trajectory 輸入 | 三個 `JointTrajectory` 訂閱：`arm_controller/joint_trajectory`(joint1-6)、`gripper_controller/joint_trajectory`([joint7])、`joint_trajectory`(joint1-7)，QoS depth 10 | `:422-441` |
| 首點驗收 | 要求 `time_from_start` 嚴格遞增且落在 (0,30]s，**t=0 首點會被拒**（MoveIt 慣例會送 t=0 起點） | `:384` |
| 插補 | 全部 waypoint 都會跑，但用逐段 smoothstep＋牆鐘 `time.monotonic()`，非線性／三次插補，非 sim 時間 | `:116-128, :403, :456` |
| 取消／逾時 | 無 cancel 介面；新訊息直接取代該 channel 計畫；完成只有一行 log，無 result／tolerance 判定 | `:402-413, :465` |
| 驅動方式 | 隱式 PD 位置驅動：arm 800/80/100，gripper joint[78] 2000/100/10 | `:165-171, :467-469` |
| 物理步頻 | dt=1/120，render_interval=4，每迴圈 2×`app.update()`+`sim.step()`+`sleep(1/120)`；無 `/clock` 發佈者 | `:140-145, :491` |
| JointState | `/robot129_sim/joint_states`，joint1..8，約每 4 幀發一次（≈30Hz 名目），系統時鐘 timestamp，無 effort，`frame_id` 空 | `:246, :472-475` |
| TF | 只有 `world→camera_color_optical_frame`；無 robot_state_publisher | `:303-314` |
| 場景物件 | 只有地板＋**純視覺**紅色 3.5cm marker（無碰撞、無剛體），供 viewport 紅像素探針用；**沒有任何可抓物體** | `:96-97, :147-159` |
| FollowJointTrajectory action server | **不存在**；grep `sim/`, `tools/`, `research/`, `deployment/`, `ros2_ws/src`（不含 external MTC）只找到 action *client*（`verify_ros2_control_guards.py`、`verify_moveit_fixed_plan.py`），皆針對 mock controller_manager，不連 Isaac | 見 `docs/LOCAL_ROS_CONTROL.md:133`、`docs/BEGINNER_TUTORIAL.md:356` |
| 夾爪 | 只接受 `joint7`；`joint8` 每迴圈強制 `-joint7`；USD 有 `NewtonMimicAPI` 但 PhysX 是否遵循 UNVERIFIED；無寬度↔關節映射程式，`calibration/gripper_mapping.active.template.yaml` 狀態 UNRESOLVED | `:34, :431, :466`；`robot129.usda:1273-1282` |
| GPU 選擇 | `start_robot129_ros_webrtc.sh` 依 `nvidia-smi` 自動選負載最低的卡（目前為 GPU0，GPU1 被其他使用者占用 95% util） | `tools/start_robot129_ros_webrtc.sh:24-27` |
| WebRTC port | 49100 目前被你自己的 `parcel-forge/view_scene.py`（PID 3859986）佔用；runner 啟動腳本會直接失敗 | 本輪 `ss -ltnup` 實測 |

**結論**：control chain 只到「topic→Isaac」，缺 FollowJointTrajectory adapter；場景缺可抓物體；runner 插補與 t=0 驗收都要改，才能接 MoveIt。

## 2. MoveIt 2.12.4 / MTC ros2-0.1.4

| 欄位 | 現況 | 證據 |
|---|---|---|
| 套件 | `robot129_description`、`robot129_moveit_config`、`robot129_sim_bringup`、`robot129_tasks`、`external/`(MTC C++-only + py_binding_tools) | `ros2_ws/src/*` |
| MTC build | 只建了 core/stages/msgs；**capabilities（含 ExecuteTaskSolution）未建置**，無 Python bindings，無 `moveit_py` | `external/README.md:5-8`；`patches/mtc-0.1.4-jazzy-cpp-only.patch:9,17-19` |
| SRDF | `arm`=base_link→tcp；`gripper`=joint7,joint8；eef `robot129_gripper` parent_link=tcp，**無 parent_group**；碰撞矩陣只含相鄰 link 與 link7/8，非完整矩陣 | `robot129.srdf:2-17` |
| kinematics | KDL，resolution 0.005，timeout 0.1，只設 `arm` | `kinematics.yaml:1-4` |
| moveit_controllers.yaml | `arm_controller`/`gripper_controller` 皆 FollowJointTrajectory，**缺 `moveit_controller_manager` key**（WARN） | `:2-4`；`out/lesson_09/move_group.log:37` |
| 既有 MTC demo | `robot129_tasks/src/robot129_mtc_plan.cpp`：CurrentState + 7 個寫死關節 MoveTo，**無方塊、無 IK、無 attach、無執行**，還自己發假 `joint_states` | `:22-40` |
| move_group namespace | 跑在根 namespace，用絕對 `/move_action`；與 `/robot129_sim` 下的 controller/joint_states 是否能對上 UNVERIFIED | `move_group.launch.py` |
| 現有結果 | 固定目標規劃 PASS（22 點，終點誤差 4.95e-4 rad）；MTC 8-stage PASS（1 個解，但只是關節插值，非真正 pick） | `out/lesson_09/*.json` |

**結論**：MTC 的 stage、solver（GenerateGraspPose、ComputeIK）都已編譯好可用，但**沒有真正的 pick task**；SRDF 需要補 eef parent_group 與碰撞矩陣；move_group 需要搬進 `/robot129_sim` namespace 且讀 live joint_states（現有假 joint_states 發佈者要移除）。

## 3. 物理抓取 demo 與 research pipeline

| 欄位 | 現況 | 證據 |
|---|---|---|
| `verify_robot129_physics_grasp.py` | 方塊 3.5cm/0.08kg，摩擦 static 4.0/dynamic 3.0（"max" combine，不真實），**漂浮在 (0.38,0,0.45) 且關重力**，夾指閉合後（frame 105）才開重力；寫死關節角 `home_arm=[0,1.20,-1.25,0,0.15,0]`、`lift_arm=[0,1.35,-1.75,0,0.15,0]`，無桌面、無接近、無 IK | `:158-165, :180-197, :230-239, :298-299` |
| 接觸 | 真實：ContactSensor 過濾方塊，`kinematic_attachment: False`；峰值約 10.3N/10.2N | `:198-205, :363` |
| 成功判定 | 雙指 >0.1N＋方塊峰值抬升 ≥3cm＋釋放後下降 ≥3cm | `:349-359` |
| 「從支撐面夾起」 | **從未被驗證**——demo 全程沒有桌面，方塊本來就懸空在夾爪間 | 同上 |
| research/src/mpg | `schema.py`(Plan v0)、`lifting.py`(反投影/RANSAC 桌面/`refine_point`)、`scene_bundle.py`(`transform_matrix`)、`candidates.py`(**2D 編號網格，非 grasp generator**)；108 個 unittest 全過 | `out/final_validation/research_tests.log` |
| IK/FK 程式 | 無 Python IK；只有 `tools/verify_fk_consistency.py` 的手寫 FK；Isaac Lab venv 內有未用的 `differential_ik.py`/`pink_ik`/`cumotion`/`pinocchio 3.9.0`/`pink 3.3.0` | 見 survey |
| `docs/interface_contract.md` | Plan3D 草案，明示 DESIGN ONLY | `:1-3` |
| `docs/action_generation_survey.md` | **workspace 中不存在**（研究計畫文件引用，但檔案未找到），其建議狀態視為 UNVERIFIED | grep 全庫無結果 |

## 4. 離線可達性分析（本輪新增，`tools/analyze_top_down_reach.py`）

用 numpy 依 `robot129.urdf` 關節原點做正向運動學（不依賴 ROS/Isaac），對 joint2–5 做網格掃描（joint1 只整體繞世界 z 軸旋轉，不影響半徑／俯視對齊，故固定為 0），找出由上往下（夾爪逼近軸與世界 -z 夾角 ≤11.5°）可行的 (半徑 r, 高度 z) 組合。

**正確性驗證**：用 `out/lesson_06/gripper_probe.json` 的實測 HOME 關節角回代，FK 算出的 `gripper_base` 位置與 Isaac 實測值誤差 <1e-6 m。夾點偏移量（gripper_base 沿逼近軸 0.125 m）與 `out/lesson_06/grasp_pose_search.json` 獨立量測的有效接觸範圍（0.108–0.138 m）吻合，可信。

結果（`out/grasp_motion/s0/reach_scan.json`）：

| 夾點高度 z | 可達半徑 r |
|---|---|
| 0.0175 m（方塊放在底座平面） | 0.003 – 0.534 m |
| 0.05 m | 0.034 – 0.512 m |
| 0.09 m | 0.081 – 0.484 m |
| 0.12 m | 0.115 – 0.452 m |
| 0.15 m | 0.170 – 0.415 m |

由上往下抓在 z=0.0175–0.15 m 的範圍內都有解，比先前設計審查 agent 的悲觀估計（宣稱 0.12m 以上無解）更寬，因為本次用了 11.5° 容忍角且用 0.125m 而非 0.18m 當夾點偏移；**採用本結果**，因為它已對照 Isaac 實測 FK 校驗過。

**採用的場景參數**（半徑、高度皆落在可達範圍內）：
- 支撐面／方塊放置面：z = 0（等同底座平面高度，即手臂裝在與桌面等高處）。
- 方塊：3.5 cm 邊長，中心 (0.32, 0.00, 0.0175)，r=0.32（落在 0.003–0.534 可達範圍）。
- 放置點：(0.27, −0.12, 0.0195)，r≈0.296（同樣可達）。
- 兩者皆在腕上相機 HOME 視角內（HOME 視角涵蓋 x∈[0.24,0.45], y∈[-0.15,0.15]，來源：設計審查 agent 讀 `probe_robot129_gripper.py` 結果，本輪未重新量測，標 UNVERIFIED-CARRIED）。

## 5. 三個會改變設計的發現（摘要，完整推理見計畫檔）

1. **`tcp` frame 偏移錯誤**：URDF `gripper_to_tcp` = 0.180 m，但指尖 mesh 只到 0.136 m（`link7.STL` 沿逼近軸範圍 0.059–0.136 m，joint7 origin z=0.1358 m 為指根）。有效接觸範圍（獨立量測）落在 0.108–0.138 m。**解法**：MTC 內用 `pinch_center`（gripper_base+0.125m）當 IK frame，不改 URDF/USD。
2. **joint5 限制 ±1.22 rad**：不阻擋 z≤0.15m 的由上往下抓（見上表），但更高處會受限。**解法**：支撐面固定在 z=0，接近／抬升量抓 4–6 cm。
3. **WebRTC port 占用**：開發期用 headless＋錄影；最終即時展示需你先關閉 parcel-forge viewer。

## 6. 下一步

進入 S1：改 runner（接受 t=0、線性/三次插補、pick_place 場景、物體 GT pose/contact topic、reset service、headless 錄影）＋ 新建 `robot129_sim_execution` FollowJointTrajectory adapter package ＋ SRDF/MoveIt 設定修正。
