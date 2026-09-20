# Lesson 10 — MPG 與端到端模擬

狀態：PASS（deterministic replay backend）；vLLM smoke PASS；live scene grounding UNVERIFIED

## 概念與架構

本課把感知、幾何、規劃、控制與 physics 的成功條件分開記錄。replay backend 固定 seed 與輸入，適合先驗證座標、資料契約與失敗歸因；live VLM 則是後續模型品質測試。

## Done／Verified

- 使用真正 Isaac 640×480 RGB-D SceneBundle，而非 synthetic fixture。
- 從紅色目標得到 bbox `[218,243,251,276]`，三個 stage points 全部取得 metric camera point 並轉到 world frame。
- SceneBundle、TF、red pixel、3-stage localization、camera/world geometry 全部 PASS。
- replay 嚴格記錄 `actual_model_calls=0`、`robot_commands=0`；輸出由 `research/out/robot129_sim_replay/latest.json` 指向。
- MoveIt 固定規劃、ROS-to-Isaac 追蹤與 PhysX lift/hold/release/settle 均有各自 PASS 報告。
- 最終影片：`out/final_demo/robot129_full_simulation.mp4`。

## NOT RUN／UNVERIFIED

- 已建立獨立 vLLM 0.29.0 user-space 環境並快取 `Qwen/Qwen3-VL-8B-Instruct`。文字與單張影像 smoke test 通過，證據在 `out/vllm_robot129/smoke_test.json`；服務測試後已停止。
- Qwen 在本系統的角色是上層決策／VLM 學習，不是手臂 joint controller。手臂收到規範化 task command 時不需要 Stage 1。
- live SceneBundle grounding 已成功呼叫模型，但使用「release pose」時畫面沒有可辨識目的地，strict schema 拒絕結果。因此完整 live grounding 仍為 UNVERIFIED。
- replay 的顏色式 grounding 是資料與幾何驗收基線，不是語意模型 accuracy benchmark。

## 下一課

以 simulation-only adapter rehearsal 固定搬回 Thor 的資料與安全契約。
