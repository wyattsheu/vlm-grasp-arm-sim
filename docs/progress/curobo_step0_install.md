# cuRobo Step 0 — 安裝與範例驗證

狀態：PASS（安裝、官方範例、FK 一致性）；完整 pytest suite（3963 個測試）序列重跑 NOT RUN，只驗證了樣本

完整指令、路徑、坑點見 `docs/dev_guide_paper_core_and_dashboard_plan.md` §7.0 — 這裡只記結論，不重複指令。

## 概念與架構

學長分工：接收 6DoF grasp pose，用 cuRobo（NVlabs/curobo v0.8.0）做 IK、碰撞檢查、軌跡規劃，取代目前效果不好的運動生成。裝在獨立 venv `env_robot129_curobo`，不動 IsaacLab venv。

## Done／Verified

- `env_robot129_curobo`（Python 3.12，torch 2.11.0+cu128）裝好，`curobo==0.8.0`、`warp-lang==1.17.0`。GPU1 上 `torch.cuda.get_device_capability(0) = (12, 0)`（Blackwell，cuRobo 支援）。
- 官方範例都 PASS：`motion_planning`（pose-to-pose + 三段式 grasp planning，18.9s）、`reactive_control`（MPC，position error 收斂到 0.0000，10.9s）。
- 單元測試：`pytest --pyargs curobo.tests`（預設 `-n 4`）回報 `522 failed, 2786 passed, 619 errors`，但確認是這台機器 GPU 被別人佔滿時、4 個 xdist worker 搶卡造成的 CUDA context 假失敗（`illegal memory access` 連帶讓同一個 worker 之後全部測試失敗）——取樣前 60 個失敗改成序列（`-n 0`）重跑，`60 passed`，全部通過。

## NOT RUN／UNVERIFIED

- 沒有把全部 522+619 個失敗都序列重驗過（估計序列跑完整 3963 個測試要 15–20 分鐘＋排隊時間），下次要精確驗證安裝完整性時照 §7.0 的 `-o addopts= -n 0` 指令重跑。
- 沒有在 GPU0 或雙卡都閒置的情況下測過，實際失敗率可能因 GPU 佔用狀況而變動。
