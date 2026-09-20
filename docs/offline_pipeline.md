# 離線實驗管線與重現方式

此管線讀取已存在的影像或 SceneBundle，不控制機器人。預設 replay 僅測程式；要實際模型推論須明確選 `--backend local` 或 `gemini`。請遵循既有 phase/場景規約。

## 一鍵驗收（不需模型、ROS 或 API key）

```bash
TMPDIR="$PWD/.tmp" PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src python3 scripts/validate_offline_pipeline.py
```

輸出在 `out/validation/<UTC timestamp>/`。最近一次成功路徑記於 `logs/offline_acceptance_20260912.json`。內容有合成 SceneBundle、四種 replay 輸入、四種 plan/overlay、surface lifting、評估 manifest、逐 command 驗收記錄。**沒有實際模型推論，圖上的點是預先寫入的 fixture。**

## Grounding

```bash
python3 scripts/run_grounding.py \
  --image data/scenes/S1_01/rgb.png \
  --instruction 'put the red cup in the bowl' \
  --scene-id S1_01 --replicate-id r0 \
  --mode ab --backend local
```

`--mode single` 為單次 structured plan。`--mode candidates` 為固定格點候選；加 `--stage-a <先前run>/stage_a.json` 可固定 A 做 paired localization。runner 要求 checkpoint 旁有 `input.json`，並驗證 image hash、instruction、scene ID 一致；新 run 同時記錄 Stage A 檔案 hash。

`--backend replay --replay responses.json` 讀一個依序回覆清單，不連網。`--replicate-id` 必填；真實 consistency repeats 使用不同 ID。重跑相同 ID 可讀 cache，不能當新樣本。

每次新 run 保存 `input.json`、`config.json`、逐次 `request_*.png/.txt`、`response_*.txt`、`requests.json`、`plan.json`、`overlay.png`、`result.json`。格式錯誤／transport error 也保留已取得的證據。exception 只記 class，避免 transport 訊息夾帶 secret。

## 舊流程基準

```bash
python3 scripts/run_baseline.py \
  --image data/scenes/S1_01/rgb.png --scene-id S1_01 --replicate-id r0 \
  --pick-instruction 'grasp the red cup' --place-instruction 'place in the bowl' \
  --backend local --output out/runs/baseline_S1_01_r0
```

保留 `baseline_copied.txt` 的 system/user 分工與 W/H 座標，兩次獨立單點查詢，不生成 waypoint。指令拆解由 experiment manifest 顯式提供，不能使用新方法推論的 roles 偷助 baseline。REST transport、對 malformed/nonfinite response 的錯誤處理不同於原 SDK client，差異存於 `input.json`，不能稱逐位元完全重現。local backend 是同 prompt 的模型比較，並非旧 Qwen→Molmo two-stage baseline。

## 3D surface lifting

```bash
python3 scripts/run_lifting.py --scene data/scenes/S1_01 \
  --plan out/runs/某次run/plan.json --output out/runs/某次run/surfaces.json
```

檢查 SceneBundle hashes、來源 image hash、camera K、畸變與 frame。非零 D 目前拒絕，需先完成正確 rectification adapter。浮點 depth 必須明確為公尺；uint16 使用已記錄 scale。TF 缺失時只輸出 camera-frame surface，base 欄位 null。每個點保持 `geometry_status=UNKNOWN`、`ik_status=NOT_RUN`；沒有自动產生 grasp offset、object/TCP pose、transport path 或 collision pass。

## 評估

準備 JSON manifest，每列為 `run_dir`、`labels`、`group`、`scene_id`；路徑相對 manifest 所在位置，且列出全部預定 runs，包括失敗與未生成的 runs。

```bash
python3 scripts/run_eval.py --manifest data/evaluation_manifest.json --output out/evaluation.json
```

目前實作粗略 target/destination bbox 命中、parsed、missing、distinct scenes；缺失/失敗保留在有標註的分母，未標註為 null。重複 run path 拒絕；同 group 不允許混合 replay 與真推論。尚未實作 grasp-region、多次定位一致性、bootstrap interval、provider token usage 或真實 3D error，不要把這份輸出當完整 Phase 4。

## Backend ledger

每個真實 attempt 送出前先落盤保留預算，結束追加 `completed/failed`；同一 `request_id` 串接兩事件。**計算 calls 時數 `event=attempt`，不能數 JSONL 行數。** 崩潰後的 pending attempt 仍占預算，避免重啟繞過 cap。`phase_id` 分開預算，無 phase 欄位的舊 log 保守算入。

共用同一 ledger 的 instance/process 以 `flock` 協調，lock 持有到 transport 完成；不保證其他未使用此 ledger 的程式也遵守 concurrency。cache key 已包含 backend、model、endpoint、system prompt、generation settings、image、prompt、replicate ID；部署相同 model name 但不同權重仍需清楚版本標記或新 cache namespace。
