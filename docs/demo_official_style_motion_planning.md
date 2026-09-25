# Robot 129：cuRobo 多目標避障與插入棒子重規劃

這兩個測試沿用既有的 Isaac Sim `/robot129_sim`、ROS `JointTrajectory` 控制橋接器，以及專案已整合的 cuRobo `MotionPlanner`。沒有把既有控制器改成 MPC。舊的 `reactive.py` 仍是 MPC + 深度 ESDF 管線；本頁兩個測試使用完整軌跡的 MotionPlanner，並在棒子出現時**明確**更新碰撞世界及重規劃。這是已知幾何與 ROS 事件觸發的重規劃，不是從深度影像辨識棒子，也不是執行中的連續 MPC 修正。

從專案根目錄依序單獨執行；每個命令會先停止前一個場景並重啟 Isaac：

```bash
cd /mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913
bash tools/run_demo_scenario.sh static_multi --webrtc --keep-running
bash tools/run_demo_scenario.sh dynamic_bar --webrtc --keep-running
```

兩個 demo 保留 120 Hz physics/contact，但把 RTX 相機和 WebRTC 顯示降為 15 FPS，demo 相機解析度為 480×360、WebRTC 為 960×540，並使用 DLSS Performance，以降低共享 GPU 上的渲染成本。若其他工作同時占滿 GPU，`observed_real_time_factor` 仍可能遠小於 1，畫面會比模擬時間慢；可先執行 `nvidia-smi`，再用 `ROBOT129_GPU=0` 或 `ROBOT129_GPU=1` 固定較空的卡，例如：

```bash
ROBOT129_GPU=1 bash tools/run_demo_scenario.sh dynamic_bar --webrtc --keep-running
```

`static_multi`：紅色實體角錐中心 `(0.20, 0.39, 0.25)` m、底半徑 `0.06` m、高 `0.50` m；cuRobo 用包住角錐的 `(0.12, 0.14, 0.50)` m 長方體作保守碰撞模型。四個 FK 驗證過的六軸目標在 `research/configs/demo/official_static_multi.yaml`，執行 `A(左)→B(右)→C(高、角錐後方)→D(低)→A`。目標是在 Isaac 中的藍色視覺標記，並非可抓取物。啟動腳本以 `--speed-multiplier 8` 執行；15 倍實測曾讓 link7 在最後回 A 時產生 38.8 N 接觸，已棄用。程式另外規劃一條**沒有角錐**的 A→B 基準軌跡，使用所有 cuRobo 碰撞球檢查它是否會與實體角錐相交，並比較避障後的末端路徑。只有全部目標到達、基準軌跡確實被角錐擋住、避障路徑有明顯差異、實際路徑對實體角錐留有間隙、接觸感測器收到資料且力小於 0.01 N 才 PASS。報告另保留保守 box 的實際路徑 clearance 作診斷，但 box 空角的負值不代表碰到錐體。

`dynamic_bar`：LEFT/RIGHT 兩個 3 cm 藍色 cube 位於機座兩側 `Y=+0.418/-0.418 m`，相距約 `0.835 m`；設定在 `research/configs/demo/official_dynamic_bar_targets.yaml`。夾爪每次到點都做 0.5 秒 close/open；cube 是視覺 grasp target，不搬動。手臂從 HOME 到 LEFT 後，先完成六次 `LEFT↔RIGHT` 無障礙轉移。停在 LEFT 後，測試程式呼叫 `/robot129_sim/insert_bar_obstacle`；Isaac 將場外停放的 `0.42×0.06×0.06 m` 紅色水平 bar 移到中央 `[0.30, 0.00, 0.48] m`，並由 `/tf` 確認已到位。此時用原本沒有 bar 的 LEFT→RIGHT 軌跡確認會相交，再呼叫 `planner.update_world(Scene(...))`，從目前 `/joint_states` 重新規劃到相同 RIGHT 目標。只有六次轉移和每次夾爪動作完成、舊路徑被 bar 擋住、世界更新及重規劃成功、新舊路徑差異至少 2.5 cm、全手臂間隙大於 2 mm、九個受監測部位無接觸、目標仍到達才 PASS。

兩個場景都透過既有 `/robot129_sim/arm_controller/joint_trajectory`、`/robot129_sim/gripper_controller/joint_trajectory` 執行；測試會等 Isaac 發布 `COMPLETE` 事件，再從實際關節回授計算 6D 末端誤差，不以「命令已送出」當完成。`robot129.yml` 的碰撞球涵蓋 `base_link`、`link1`–`link8` 和 `gripper_base`；Isaac 對 `link1`–`link8`、`gripper_base` 與測試障礙物發布獨立接觸力。報告會保存各段規劃成功與時間、完整關節命令與實際軌跡、軌跡時長、終點位置/方向誤差、最小幾何間隙、感測器最大力及樣本數，並記錄每段 `observed_real_time_factor`。動態測試再保存插入模擬時間、TF 確認、世界更新時間、重規劃時間、舊/新 LEFT→RIGHT 關節軌跡，以及末端路徑的最大/平均偏差。

每次的輸出在 `out/demo/<場景>/<UTC時間>/`：`report.json` 是機器可讀驗收結果；`demo.log` 是執行摘要；`dashboard.rrd` 保留 3D 目標、感測障礙物、規劃與實際路徑及四格相機；`video.mp4`、`scene_video.mp4`、`wrist_video.mp4` 是錄影。Rerun 只顯示四格主畫面，Joints、Contacts、Status 下方圖表均已移除；碰撞數值仍保存於 report。兩個正式場景的 3D 視窗都不畫理想 cone/bar 輪廓；橘點是 wrist/rear RGB-D 深度反投影後，在障礙物附近 ROI 擷取的實際感測點。cuRobo 內部仍使用已知保守碰撞幾何來規劃。目標只顯示簡短英文 `A/B/C/D` 或 `LEFT/RIGHT`。灰線＝原本無障礙的基準路徑，藍線＝目前規劃軌跡，綠線＝Isaac 實際末端軌跡，紅線＝bar 出現後的新軌跡。

用以下指令檢查最近結果：

```bash
run_dir=$(ls -dt out/demo/dynamic_bar/*/ | head -1)
python3 -m json.tool "${run_dir}report.json" | less
```

要比較棒子前後的數值，查看 `old_trajectory_against_new_world.minimum_clearance_m`（負值才表示舊路徑被擋）、`legs[-1].planned_minimum_clearance.minimum_clearance_m`、`legs[-1].execution.actual_minimum_clearance_m`、`path_change.maximum_ee_deviation_m`；`old_trajectory_joint_rad` 和 `new_trajectory_joint_rad` 是完整可供作圖的六軸樣本。靜態測試看 `legs[1].unobstructed_path_against_physical_cone`、`legs[1].detour`、各段 `execution`。

WebRTC 模擬畫面使用啟動命令列印的 `WebRTC=...:49100` 位址，由 Isaac Sim Streaming Client 觀看。原先的四格 Rerun 網頁仍是：

`http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy`

若瀏覽器與模擬器不在同一台電腦，先建立 `ssh -L 9090:localhost:9090 -L 9876:localhost:9876 <server>`。執行結束使用 `--keep-running` 會先停止寫入並封存該次 `dashboard.rrd`，再以 `--no-save` 重啟相同網址的即時 Rerun；Isaac/WebRTC 會繼續運作，但封存檔不會因長時間檢查而無限增長。歷史規劃與實際軌跡請回放該次 `dashboard.rrd` 或影片；即時 Rerun 用來檢查保留中的最終場景。下一個場景命令會自行清掉上一個。三個視角攝影機和 3D 軌跡在 Rerun；WebRTC 視窗可直接看手臂、標記與實體障礙物的運動。
