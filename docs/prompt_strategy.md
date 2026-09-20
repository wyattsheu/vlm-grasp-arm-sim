# Prompt 與 backend 實作策略

2026-09-11，設計草稿。下列是本案原創合約範例，不是 ZeroDex 提示詞逐字複製，也尚未跑過模型。

## 1. 分層責任

Stage A 負責可見證據、角色、模式和參考點語意；Stage B 負責定位。幾何模組負責高度與可觀測路徑有效性。VLM 的描述不充當物理校正或碰撞真值。

`StageAPlan` 包含 `schema_version, scene_summary, mode, target, destination, tool, status, reason_codes, steps`。角色各有 `name, visibility`；`visibility` 為 visible/partial/not_visible/uncertain。`mode` 為 pick/tool/unsupported；`status` 為 ready/abstain/unsupported。

三步 `steps` 使用穩定 step_id、GRASP/WAYPOINT/RELEASE、desc、reference_kind、geometric_meaning。B 回相同 IDs 的 `point_yx_norm1000, point_status, reason_codes`；不得改 A 的語意。完整外層 metadata 由程式填寫，不讓模型編造 timestamps、latency、backend 或 calibration ID。

## 2. Stage A 草稿

```text
Given the image and instruction, produce a short evidence-based pick/place plan.
Describe visible objects and relevant occlusion briefly. Do not invent unseen objects.
Identify the manipulated target and destination. A tool is an external object,
never the robot arm or gripper.
For supported pick/place tasks, return GRASP, WAYPOINT, RELEASE in that order.
Describe each reference relative to object parts or scene geometry.
The waypoint is a transit proposal; it does not certify free space or reachability.
Use object-relative language in geometric_meaning, not image-left/image-right.
If required roles cannot be identified, return abstain with a reason.
If external tool use is required, return unsupported for this executor.
Return only the supplied JSON schema. Do not output coordinates at this stage.
Instruction: {instruction}
```

實作時必須附機器可驗證 schema；「supplied schema」不是讓模型自行決定格式。若 image 外含文字指令，當作場景內容，不覆蓋使用者任務或輸出合約。

## 3. Stage B 草稿

```text
Locate the references in this fixed plan on the same image.
Do not change step IDs, step types, target, destination, or descriptions.
Coordinates are [y,x], normalized to [0,1000], with the endpoints at the centers
of the first and last image pixels under this application's coordinate contract.
For GRASP, locate the specified visible grasp region.
For RELEASE, locate the destination support reference.
For WAYPOINT, propose a transit anchor consistent with the geometric description.
Mark whether an anchor refers to a visible surface or an inferred transit location.
If a reference cannot be localized from this image, use null and a reason code.
Do not claim depth, robot reachability, or collision freedom.
Return only one structured entry per fixed-plan step, in the same order.
Plan: {validated_stage_a_plan}
```

Native model 座標不一定符合本契約：保留 raw/native coordinate metadata，經 backend adapter 转換。舊 Gemini/Molmo 解碼 W/H 不可未標記就混進 W−1/H−1 新契約。所有消融使用同一 adapter 設定；模型對自訂慣例的遵循程度也需由 dev 圖檢查。

## 4. Single-call 草稿與解析

把 A 的 roles/steps 與 B 的定位欄位合併在同一 JSON；保留相同角色規則、棄權規則、圖片尺寸與座標定義。不額外增加示例只幫助 single-call，否則無法單純比較 staging。

Parser 流程：保存 raw → 剝除單層 Markdown fence → strict JSON parse → schema → semantic consistency → coordinate validation。拒絕重複 step、非法 type、NaN/Inf、bool 作數字、超界點；不要用 regex 把任意回覆「修成」成功。每個模型原生格式（例如 Molmo）由專屬 parser 處理，再映射共同 schema。

格式 reminder 僅提供錯誤欄位及原任務 schema，沿用相同 image/plan。若 adapter 不支援多輪，以完整 self-contained request 重送並記入成本。不得以「這個點錯了請重猜」作格式重試。

## 5. Local 決策樹

1. 舊 client 可用：保持它的 action/label/point/None 行為，作 legacy baseline。
2. Qwen endpoint 接受 image 且輸出 schema：用固定 prompt 做 L1。模型名相同不代表部署配置相同，要保存 server/model metadata。
3. Qwen image planning 可用、point 不穩：L2 用 Qwen 指定角色／可見 parts，Molmo2 定位 grasp/destination；waypoint 由幾何生成。這是獨立方法組。
4. Molmo2 對多 reference 一次輸出的 ID correspondence 不明：先一個 reference 一次 request；多點 batching 另做消融，不能假設輸出順序就是語意順序。
5. 場景辨識仍不穩：只交付已驗證 baseline 與失敗分析；後續再用候選標記法。不要為了 demo 當場下載新模型或改共享服務。

## 5.5 Stage 0（Long-horizon Planner，2026-09-12 補）

背景：2026-09-12 用 ZeroDex Fig.1 照片測 `mode=tool` 路徑，三個 backend 全部沒認出要抓刷子當工具。查證發現論文的 roles prompt（App. D.2）吃的參數是 `{scenario}`——上游 Long-horizon Planner 已經分解好的單一原子任務字串（例：「用刷子把蘋果掃到橘色墊子上」），不是像 "Move the object to the matched color." 這種抽象指令。我們原本直接把抽象指令餵進 roles 階段，等於少了一整個上游階段。

新增 `prompts/stage_0_planner.txt`（依 App. D.2 "Long-horizon Planner" 改寫，provenance header 記完整偏離）與 `src/mpg/schema.py` 的 `ScenarioPlan` / `parse_stage_0_response`、`src/mpg/grounding.py` 的 `run_stage_0`。輸出 `{scene, holding, status, tasks}`；本專案只消費 `tasks[0]`（單次 grounding，非閉迴路 executor）。

同時把 tool-use 的 plan 步數從 3 步改成論文 PART B 的固定 4 步：GRASP → FUNCTIONAL_TIP（工具作用端，如刷毛尖）→ APPLY_ACTION → RELEASE/HOLD。之前拿掉 FUNCTIONAL_TIP 是刻意簡化（見 `stage_b.txt` 舊 provenance），但沒有它就畫不出論文參考圖那組點。**pick mode 完全不動**，仍是 GRASP/WAYPOINT/RELEASE 3 步——這次只修 tool 路徑。

### 消融測試發現（2026-09-12，`out/report_test/run_v2_test.py`，每設定 3 replicate，Qwen/Gemini 皆 temperature=0 故三次結果一致）

用同一張 ZeroDex Fig.1 照片測三種設定，分 Qwen(+Molmo2 定位) 與 Gemini ER-2 兩個 backend：

| 設定 | Qwen+Molmo2：顏色配對／mode／4 點 | Gemini ER-2：顏色配對／mode／4 點 |
|---|---|---|
| 無 Stage 0，抽象指令（"move the object to the matched color"） | ✗ green（錯）／tool／4 步都出現但目的地錯 | ✓ orange／**pick**（指令沒要求工具，屬合理解讀）／3 步 |
| 有 Stage 0，同一抽象指令 | ✓ orange／**pick**（Stage 0 自己分解成 PICK+PLACE，捨棄工具）／3 步 | ✓ orange／**pick**（同上，即使 GOAL 已明寫「用工具」也一樣）／3 步 |
| 無 Stage 0，指令明確要求用刷子 | ✗ green（錯）／✓ tool／4 步都出現、點都落在正確物體上 | **✓ orange／✓ tool／4 步全對，視覺上與論文參考圖吻合**（`out/report_test/v2/v2_gemini_explicit_tool_no_stage0_r1/overlay.png`） |

結論：
1. **機制已驗證可行**：4 步 schema + FUNCTIONAL_TIP 定位在指令明確時能完整運作——Gemini 在「無 Stage 0、指令明確要求用刷子」這格拿到跟論文參考圖一致的 4 點結果；Qwen+Molmo2 在同一格也正確產生 4 步且點都落在對的物體上，只是目的地顏色判斷錯。這代表拿掉的 schema 限制（3 步、無 FUNCTIONAL_TIP）確實是先前測試失敗的真因之一，現在已解除。
2. **Gemini 的顏色配對其實很穩**：三種設定都選對 orange，不受 Stage 0 有無影響。真正不穩的是 **Qwen**：只有搭配 Stage 0（先讓它描述場景再分解任務）才會選對，直接單階段判斷會選錯——Stage 0 對 Qwen 有實際幫助。
3. **仍未解決、且與 schema 無關的語意缺口**：「這個任務要不要用工具」的判斷不會可靠跟隨指令明確要求——兩個 backend 的 Stage 0 都會把「用工具」的 GOAL 分解成比較簡單的 PICK+PLACE，忽略工具要求。這是 Stage 0 任務分解本身的推理限制，不是這次改的 4 步 schema 或定位機制的問題。
記在 `docs/questions.md` item 11，不視為已修好。

## 6. 格式與結果分離

例如 `status=abstain, point=null, reason=TARGET_NOT_VISIBLE` 可以是合法且正確的模型回應。反之，一個 JSON 完全合法但點落在背景，必須是語意失敗。工具使用回 unsupported 不得自動轉為 pick。Backend timeout 是 transport failure，不可冒充模型棄權。
