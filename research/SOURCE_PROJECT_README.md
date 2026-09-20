# Robot 129 多點 VLM Grounding

準備開工請先讀 [完整執行手冊](docs/execution_playbook.md)，包含第一個工作時段、Phase 0–5 任務順序、交付與驗收標準。

| 文件 | 用途 |
|---|---|
| [執行手冊](docs/execution_playbook.md) | 每階段做什麼、如何驗收、卡住如何縮減範圍 |
| [研究方案與架構](docs/research_plan.md) | 為何採用這個方案、模組與架構圖 |
| [資料與評估規約](docs/data_and_evaluation.md) | SceneBundle、12 景切分、參數、實驗矩陣、指標與預算 |
| [Prompt/backend 策略](docs/prompt_strategy.md) | A/B、single-call 原創草稿、parser、local 決策樹 |
| [文獻到實作](docs/methods_and_references.md) | 閱讀順序、具體借用方法、備案與工程來源 |
| [論文索引](docs/related_papers.md) | 相關論文的一手連結 |
| [Phase 0 系統盤點](docs/00_system_recon.md) | live deployment、原始碼、ROS／服務狀態與缺口 |
| [待確認事項](docs/questions.md) | 開工前需對齊的部署／資料／介面 |
| [隔離 ROS 環境](docs/ros_capture_environment.md) | Wyatt 核准的 Git-ignored runner、邊界與驗證 |
| [相機恢復程序](docs/camera_recovery.md) | grasp_view 啟動鏈、實際故障根因與人員操作順序 |

細部實驗設定以資料與評估規約為準；安全與階段門檻仍以 AGENTS 為準。所有腳本名稱和參數起始值均為待實作設計，不是現成可執行工具。

Phase 0 已完成。Phase 1 的 subscriber-only capture、驗證與標註工具已實作並通過離線測試，但相機 stale 且 WORK_DIR 所在 host Python 沒有 ROS packages，所以真實擷取仍為 **NOT RUN**。推論、IK 與機器人執行也皆 **NOT RUN**。

- 規則：[AGENTS.md](AGENTS.md)，已填入三個找到的路徑，保留原始限制並前置差異修正。
- 原始附件：[original_task_spec.md](refs/original_task_spec.md)。
- 論文：[ZeroDex v2 PDF](refs/zerodex_2606.19340v2.pdf) 與 [閱讀筆記](refs/reading_notes.md)。
- 介面：[interface_contract.md](docs/interface_contract.md)。
- 後續：[future_active_multiview.md](docs/future_active_multiview.md)。
- Phase 0 紀錄：[phase report](docs/phase_reports/phase_0.md) 與 [evidence summary](logs/phase0_evidence.md)。
- Phase 1 目前狀態：[phase report](docs/phase_reports/phase_1.md)；執行方式與資料契約見 [data README](data/README.md)。

## 啟動 Codex：已核對本機 help

2026-09-11 本機版本為 `codex-cli 0.154.0-alpha.6.1`，已執行 `codex --help`，原輸出在 [logs/codex_help.txt](logs/codex_help.txt)。支援 `-C`、`--sandbox workspace-write`、`--ask-for-approval on-request`。以下是**下一次 session 的建議指令，本次未實際啟動第二個 Codex**：

```bash
codex -C /home/wyattsheu/workspaces/next_arm_tesk \
  --sandbox workspace-write \
  --ask-for-approval on-request \
  -c 'sandbox_workspace_write.writable_roots=[]' \
  -c 'sandbox_workspace_write.exclude_slash_tmp=true' \
  -c 'sandbox_workspace_write.exclude_tmpdir_env_var=true' \
  -c 'sandbox_workspace_write.network_access=false'
```

額外設定限制 `/tmp`／`TMPDIR` 與額外 writable roots；來源：[OpenAI 官方 configuration reference](https://learn.chatgpt.com/docs/config-file/config-reference)。實際環境可能有 managed policy；啟動後仍需確認有效權限。不要加 `--add-dir` 把共享 repos 變成可寫。

AGENTS 是行為規則，不能替代 OS sandbox。本次原先遇到 `bwrap: loopback: Failed RTM_NEWADDR`，使用者之後開啟完整權限；因此**本次不能宣稱已由 OS 強制限制只寫 workspace**，而是所有任務檔案操作仍遵守指定路徑。尚未診斷該 sandbox 啟動錯誤，也未改全域設定。

日後 Python 工作在本 workspace 設 `PYTHONDONTWRITEBYTECODE=1`、`TMPDIR=$PWD/.tmp`、`XDG_CACHE_HOME=$PWD/cache`、`PIP_CACHE_DIR=$PWD/cache/pip`、`MPLCONFIGDIR=$PWD/cache/matplotlib` 等，避免唯讀 import 在外部 repo 產生 pycache 或工具寫入全域快取。需要的資料夾均先建在 workspace；venv 依 AGENTS 使用 `--system-site-packages`。這些執行環境尚未建立。

此路徑邊界指 agent 執行任務的檔案／cache；Codex 應用本身的 session／認證儲存另由 host 管理，本次沒有搬移或修改。下一步依 [Phase 1 report](docs/phase_reports/phase_1.md) 補齊 camera、frame、depth scale 與 ROS capture environment；不可由 Codex 重啟共享 camera service。
# 2026-09-12：研究與離線實作更新

- [本輪進度與驗證](docs/phase_reports/offline_iteration_20260912.md)
- [離線 runner、baseline、lifting 與 evaluation 使用方式](docs/offline_pipeline.md)
- [900 個合成深度視窗的研究結果](docs/synthetic_depth_results_20260912.md)
- [方法到論文的證據表](refs/method_evidence.md) · [BibTeX](refs/references.bib)
- [代碼審查](docs/code_review_20260912.md) · [研究決策](docs/research_decisions_20260912.md)

以下原始規劃／歷史狀態保留；最新狀態以本輪報告為準。
