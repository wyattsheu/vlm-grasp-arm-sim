# 複製以下提示詞給 Codex 或 Claude Code

```text
我正在 PRO 6000 server 上學習並建立 Robot 129 的 Isaac Sim 數位分身。這個 workspace 是從 Robot 129 搬來的 handoff bundle。

請先完整閱讀：
1. AGENTS.md
2. README.md
3. LEARNING_PATH.md
4. docs/pro6000_sim_migration_plan.md
5. provenance/README.md
6. environment/version_matrix.yaml

請擔任我的實作老師和協作者，用繁體中文一步一步帶我建立。每次只進行一個 lesson，不要一次把十一課全部做完。每課先說明概念、架構位置、會改哪些檔案、要執行哪些命令與預期結果；接著和我完成操作、診斷錯誤、跑驗收，最後寫 docs/progress/lesson_NN.md，列出 Done、Verified、NOT RUN／UNVERIFIED、產物路徑和下一課。

先做這些事情：
- 執行 bash tools/verify_bundle.sh。
- 執行 bash tools/collect_server_info.sh。
- 閱讀 environment/server_environment.txt，判斷實際 GPU、VRAM、OS、driver、Docker、磁碟與 ROS 狀態。
- 根據官方相容矩陣填寫 environment/version_matrix.yaml。版本與文件若可能已更新，請查 NVIDIA／ROS／MoveIt 官方來源。
- 建立 docs/progress/lesson_01.md，開始 LEARNING_PATH.md 的 Lesson 1。

robot/vendor 是唯讀來源副本，任何修正都建立在新的 robot129_description package。robot/reference_moveit_config 只是參考設定。現有相機外參、夾爪 mapping、controller gains 和真實 camera intrinsics 尚未完整驗證，不要默認為正確。

這個 workspace 只允許模擬。不要連接 CAN、Piper SDK、RealSense USB 或 production ROS domain，不要啟動任何實機 driver。模擬運動前要先證明 ROS domain／namespace 與 Robot 129 隔離。遇到需要 sudo、驅動變更、Docker daemon 設定、大型下載或連接 Thor 時，先整理具體命令、空間／網路影響和回復方式，再由我決定。

教學重點是讓我理解 URDF／USD、link／joint／TF、camera optical frame、physics／contact、ros2_control、MoveIt、SceneBundle、VLM grounding 與 sim-to-real 的邊界。不要只替我完成而不解釋，也不要用 GUI 成功畫面代替可重現的設定與測試。
```

