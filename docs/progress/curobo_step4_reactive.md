# cuRobo Step 4 — 即時避障（Mapper → ESDF → MPC），真的跑通了

狀態：PASS（真實 Isaac 跑通 264 步 MPC、0 次碰撞，且跟「沒開控制器」的基準組做過對照）

完整指令、架構決策（跟原計畫的差異）、踩過的坑、實測數字見 `docs/dev_guide_paper_core_and_dashboard_plan.md` §7.8。

## 概念與架構

照 cuRobo 官方範例 `live_volumetric_mapping_mpc.py` 的方法：即時融合深度成 TSDF → 算 ESDF → 每個控制 tick 跑 MPC。**跟原計畫不同**：改成獨立 ROS 2 process（`env_robot129_curobo`），不是 Isaac 行程內 lockstep——因為 lockstep 需要把 cuRobo 裝進共用的 IsaacLab venv，風險太高。改用既有的 `/robot129_sim/piper/joint_cmd` 串流指令介面溝通，跟 repo 其他地方「process 之間永遠用 ROS/檔案溝通」的慣例一致。

新增 `--scene dynamic_stick`：一根不在任何規劃器已知幾何裡的木棒，正弦擺動，只能靠即時深度看到；`link3`–`link6`／`gripper_base` 上掛 contact sensor 當物理真值。

## Done／Verified

- **找到並修掉一個真的 cuRobo bug**：`CameraObservation.depth_to_meter` 預設 0.001（假設毫米深度），但模擬發布的是公尺深度，沒設會讓所有反投影點塌縮到相機前幾毫米。這個 bug 造成兩種相反的沉默失敗（手腕相機 100% 誤判、後方相機 0% 誤判），花了幾輪「合成資料→依然不對→真實資料但關節角度沒對齊→依然不對→真正同步的真實資料」才抓到，過程記在 dev guide。修法：每個 `CameraObservation` 都明確帶 `depth_to_meter=1.0`。
- 另外兩個 API 坑也修了：`RobotSegmenter.from_robot_file()` 的路徑解析限制、`ops_dtype` 預設 bfloat16 跟 float32 碰撞球衝突。
- 用真正同步擷取的（深度、相機內參、TF、關節角度）驗證過分割正確：8.4% 像素判定為機械臂，遮罩範圍精確對應畫面左上角的夾爪位置（跟 Step 2 截圖吻合）。
- 完整 pipeline（segment→filter→integrate→esdf→mpc→一步動作）串起來測過，做成 `research/tests/test_reactive_controller.py` 的回歸測試（2 個 case，用真實資料當 fixture，不是合成資料）。
- **真的用 Isaac 跑通即時避障**：`--scene dynamic_stick --scene-camera pole` 啟動，`reactive.py` 對抗跑 90 秒（wall-clock），264 次 MPC 求解全部成功（0 次「無解」），過程中取樣 4 次全部 0 碰撞力，同時關節角度持續變化（證明手臂真的在動、不是卡住才沒撞到）。
- **對照組**：控制器沒開時，HOME 姿態下 `gripper_base` 接觸力量到 297N（棒子初始位置剛好卡在手臂正下方）——證明這根棒子是真的物理威脅，不是擺好看的。

## NOT RUN／UNVERIFIED

- 沒做「關掉 Mapper、只用已知幾何」的對照組——上面的「控制器完全沒開」基準組已經算是一種對照，判斷邊際價值低，先跳過。
- 只用了後方相機；手腕相機沒有加進 Mapper（它離自己太近，分割後可能還有大部分視野是自己的硬體，需要另外驗證）。
- 沒有分開量測 segment/integrate/esdf/mpc 各階段各自的 wall-clock 延遲，只有「264 步/90 秒」的總體數字。
- 棒子擺動週期（6 秒）偏慢，沒測過更快、更難躲的狀況。
- 沒有把這個 node 包成可以對接實機的版本（架構上已經朝這方向設計，但沒有實機可測）。
