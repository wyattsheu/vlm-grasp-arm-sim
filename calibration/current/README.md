# Current verified calibration artifacts

驗證完成後的 immutable snapshot 才放在這裡，檔名包含 calibration ID 與日期，例如 `wrist_d435i_20260920_intrinsics.yaml`。頂層 `calibration/*.yaml` 是目前 active pointer/config；template 不能直接改成 `VERIFIED`。

每份 VERIFIED 檔案至少包含：來源主機、裝置／硬體 revision、frame direction、日期、操作者、原始資料路徑、誤差指標、reviewer 與檔案 SHA-256。
