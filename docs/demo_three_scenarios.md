# 三個 cuRobo 6D 位姿與障礙物 Demo

這三個場景共用現有的 `research/src/mpg/curobo_bridge/reactive.py` 之 `--waypoints` 控制流程；`tools/run_demo_scenario.sh` 只負責切換模擬場景、開啟 Rerun、錄影及收集報告。所有指令都只作用於 ROS_DOMAIN_ID=129 的 `/robot129_sim` 模擬器。

在專案根目錄，依序單獨執行：

```bash
bash tools/run_demo_scenario.sh pose5 --webrtc --keep-running
bash tools/run_demo_scenario.sh p2p_cone --webrtc --keep-running
bash tools/run_demo_scenario.sh stick_ab --webrtc --keep-running
```

每次執行會先關閉上一個場景並重啟 Isaac，因此不要同時執行三條。Rerun 一律開啟原網址：

`http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy`

如果瀏覽器與模擬器不在同一台電腦，先建立 `ssh -L 9090:localhost:9090 -L 9876:localhost:9876 <server>` 通道。四格畫面依序是 3D 目標與障礙物、全景、後方相機、腕部相機。3D 中綠點是起點、藍點是中途點、紅點是終點；同色箭頭表示指定夾爪方向，青色箭頭是目前夾爪方向。紫色輪廓是模擬障礙物的**實際位置**，橘點是 cuRobo 收到的深度佔用格，灰色淡點是原始深度背景。橘點只是深度佔用格，不等於已完成物件辨識；執行控制器前不會更新。下方 Contacts 頁籤顯示接觸力。

| 場景 | 輸入與畫面 | 驗收條件 |
| --- | --- | --- |
| `pose5` | 編輯 `research/configs/demo/five_pose_waypoints.yaml` 的五個 `position_m`（公尺）和 `quaternion_xyzw`；也可傳 `--waypoints /abs/path.yaml`。3D 依序標 P1–P5 與方向箭頭。 | 五點均在時限內到達，`reactive_report.json` 為 `PASS`，軌跡與 Contacts 無碰撞。這測試現有 cuRobo MPC 對這組 6D 目標的能力；任意不可達位姿沒有成功保證。 |
| `p2p_cone` | 使用 `research/configs/demo/ab_poses.yaml` 的 A、B 目標和固定角錐；顯示 A/B、角錐輪廓與橘色感測佔用格。 | A→B 到達，角錐附近有持續感測佔用格，接觸力維持 0 N。 |
| `stick_ab` | 目標仍是相同 A、B；以 22 cm 高棒子取代角錐，棒子中心在 `[0.2054, 0.3581, 0.11]` m、沿 Y 軸 ±1.5 cm、12 秒一週期原地擺動。 | A→B 到達、橘色佔用格隨棒子更新，接觸力維持 0 N。 |

每次輸出存於 `out/demo/<場景>/<UTC時間>/`：`reactive_report.json` 是到點結果，`reactive.log` 是控制器日誌，`dashboard.rrd` 和影片可回放。執行後用以下指令查看最近一次結果：

```bash
run_dir=$(ls -dt out/demo/pose5/*/ | head -1)
python3 -m json.tool "${run_dir}reactive_report.json"
```

把 `pose5` 換成 `p2p_cone` 或 `stick_ab` 即可查其他場景。腳本在 `FAIL` 時會回傳非零值；`--keep-running` 讓模擬器與 Rerun 保持運作，以便檢查畫面。

2026-09-24 修正閉迴路後已在 Isaac 重跑三個場景：`pose5` 5/5 到達（最大位置誤差 6.2 mm、resync 0）；`p2p_cone` A/B 到達（最大位置誤差 6.7 mm、resync 0）；`stick_ab` A/B 到達（最大位置誤差 6.7 mm、resync 0）。棒子場景的 dashboard.rrd 記到 14 組障礙體素，單次 17–33 點；棒子 TF 1,191 筆的 Y 座標範圍為 0.3431–0.3731 m，點雲平均 Y 也在 0.338–0.366 m 間變化。link3–6 和 gripper_base 各 890 筆接觸資料，最大值都是 0 N。輸出分別在 `out/demo/pose5/20260924T122647Z/`、`out/demo/p2p_cone/20260924T122900Z/`、`out/demo/stick_ab/20260924T123053Z/`。點雲變化與接觸力已由 Rerun 記錄量化確認；實際展示仍可直接開原本的 9090/9876 網址檢視錄影。
