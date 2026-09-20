# Provenance and licensing

`robot/vendor/piper_description` 是從下列唯讀來源逐檔原樣複製：

```text
/home/wyattsheu/workspaces/robotic_agent/mm_system/piper_ros/src/piper_description
```

本包只選入 `package.xml`、`CMakeLists.txt`、主要 with-gripper URDF／Xacro 和 mesh。來源 repo HEAD 由 2026-09-11 的既有盤點記錄為 `583ada7af4d4c52420d82edac34cdec919228712`，但來源工作樹當時有修改，因此以本包 `SHA256SUMS` 作逐檔身分依據。Piper 來源的 MIT license 保存在 `PIPER_LICENSE`。

`robot/reference_moveit_config` 來自同一 repo 的：

```text
piper_ros/src/piper_moveit/piper_with_gripper_moveit/config
```

它完整保留設定以供比較，並非經驗證的 Isaac 設定。不要執行其中的 mock／controller 配置，直到教學 lesson 完成檢查。

`research/` 來自 `/home/wyattsheu/workspaces/next_arm_tesk` 的 MPG 離線工作。此工作區目前沒有獨立專案 license；在擁有人確認授權前，交接包按 ACM Lab 內部研究用途處理，不對外散布。

本包排除了 `.env`、API response cache、VLM call logs、ROS build/install、Python venv、報告附件、模型權重與真實 camera serial。`prior_source_manifest.json` 是先前盤點的證據副本。

驗證方式：

```bash
bash tools/verify_bundle.sh
```

