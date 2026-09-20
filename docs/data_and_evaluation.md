# 資料、參數與實驗規約

2026-09-11，DESIGN ONLY。本文定義接下來要建立的檔案與量測，沒有虛構 dataset 或結果。

## 1. SceneBundle

每個 `data/scenes/<scene_id>/` 包含：

| 檔案 | 必填資訊 |
|---|---|
| `rgb.png` | 原 RGB；保存色彩通道慣例，模型實際 JPEG bytes 另存 run |
| `depth.png` 或 `depth.npy` | uint16 原始尺度用 PNG；float depth 用 NPY，保留 NaN，不靜默量化 |
| `camera_info.json` | width/height、K、D、distortion model、frame、header stamp、depth scale |
| `tf.json` | T_base_camera、方向、timestamp、來源；FK 時另存 T_ee_camera 與校正 hash |
| `joint_states.json` | joint names、positions、units、stamp；與 FK 順序的 mapping |
| `instruction.txt` | 唯一 primary 指令；額外改寫不能混作新 scene |
| `labels.json` | 人工標註；只供 evaluator／明示 oracle diagnostic 使用 |
| `manifest.json` | capture ID、layout ID、檔案 hashes、各 timestamp、capture_complete、品質狀態 |

資料品質最低檢查：RGB/depth 尺寸和 alignment 一致；K 合法且匹配影像；depth scale 明確；transform 4×4 最後一列合理、R 正交且 determinant 近 +1；曝光與關節／TF 時間有依據。用 round-trip project/deproject 檢查軸序，但它不驗證相機校正本身的真實精度。

同步時差門檻需依物體／相機移動速度與容許位置誤差決定。原碼 slop=50 ms 只是觀測到的設定，不能直接稱足夠；本週静態場景可先使用較嚴的採集門檻並記錄實際差值。缺曝光時間匹配 transform 的資料只准做 2D。

## 2. Scene matrix 與標註

| IDs（預計） | 類型 | 主要測項 | 切分 |
|---|---|---|---|
| S1_01–04 | 無障礙，改變物件／托盤位置 | 基本 grasp、release | 01 dev；02–04 holdout |
| S2_01–04 | 一個障礙，改變高度／路徑相對位置 | 過頂／繞行與 unknown | 01 dev；02–04 holdout |
| S3_01–04 | 顏色相近、類別相近、部分遮蔽 | 角色／ref ambiguity | 01 dev；02–04 holdout |
| N_01–03 | target 不存在、destination 不存在、不可辨識 | 正確棄權、錯誤正例 | dev diagnostics，另報 |

每景固定一條指令；12 景中直接／間接指令盡量平衡且各類都有。間接指令若人也無法唯一判斷，標 AMBIGUOUS，不硬給唯一 ground truth。後續測同圖兩種措辭要用 paired analysis，呼叫數亦翻倍，不能沿用 135 次預算。

labels 規約：bbox 用原圖 pixel `[xmin,ymin,xmax,ymax]`，連續座標邊界 inclusive，記入 annotation_version；region 用 polygon／mask。包含 target instance ID、可接受 grasp region(s)、destination acceptable region(s)、obstacle masks、visibility、instruction_validity、annotator、備註。多個合理 grasp 點可以用多個 region，不用單一「標準點」誤罰。

Ground truth 的 grasp region 由人工依夾爪與任務判讀；它仍不是物理抓取成功真值。支撐面 region 與 gripper／物體 footprint 的適配需另外檢查。

## 3. 參數策略

以下為離線開發起始值／必要待填項，**不是 Robot 129 已驗證參數，也不是物理安全保證**。開發時寫入 `configs/default.yaml`，凍結前只用 dev 場景调整。

| 參數 | 起始策略 | 來源／凍結條件 |
|---|---|---|
| coordinate mapping | 新契約 endpoint W−1/H−1；legacy adapter W/H | 本案介面決策，保存 adapter ID |
| depth window | 7；legacy 11 | 7/11 單變數消融 |
| depth statistic | median；legacy mean | 固定 window 再比較 |
| valid fraction | 0.5 作 dev 起值 | 邊界以實際取樣 patch 為分母；同時記 nominal coverage |
| usable depth range | UNVERIFIED，不猜 | 相機配置與現場量測確認；legacy 0.1–3.0 m 僅重現 |
| voxel size | 0.005 m 作 dev 起值 | 小於所需幾何特徵；薄障礙不可因降採樣消失 |
| table RANSAC threshold | 0.005 m 作 dev 起值 | depth noise 實測；平面法向／coverage 額外驗證 |
| refine radius/step | 0.04/0.02 m | ZeroDex 參考值，本機可行性另測 |
| release clearance | 0.01 m 作離線起值 | object bottom reference；不是 TCP offset |
| transport clearance | 0.03 m 作離線起值 | 另加量測／校正／模型不確定度，缺者 UNKNOWN |
| workspace bounds / object dims / gripper envelope | UNVERIFIED | Tuan 模型或量測；未填不得 PASS 完整代理幾何 |
| path sampling / collision margin | 聯動設計 | 掃掠近似至少補半步長及 voxel／量測誤差；旋轉另補角度誤差 |
| maximum retries | 格式 1 次 | 正確棄權不 retry；傳輸失敗依手冊保守處理 |
| cloud phase cap | 200 HTTP attempts | 送出前 ledger 檢查，失敗與 timeout 亦計 |

Table normal 朝向由已確認的重力／base 定義判定，不把 camera z 當上方。觀測點雲若只支持局部尺寸，unknown extent 不能用小 bbox 補成完整物體。

## 4. 公平對照矩陣

| ID | 內容 | 想回答的問題 |
|---|---|---|
| LEGACY | 舊 pointing prompt＋舊解析＋舊 geometry | 原系統的靜態重現；與新系統差異混合，單獨報 |
| B0 | 舊選點 prompt 的 grasp/release，統一新 adapter | 兩次獨立單點基準 |
| B1 | 同 Gemini，Stage A/B 多點，統一 adapter | 多點語意 planning 取捨 |
| B2 | 同 Gemini，A/B 合一次，統一 adapter | 呼叫數與準確度取捨 |
| L1 | 單一 local 模型、固定 B1/B2 選定 prompt | prompt transfer／model comparison |
| L2 | Qwen vision roles＋Molmo2 surface localization＋幾何 waypoint | 混合系統效益，不稱純模型消融 |
| G0/G1 | 固定 2D 輸出、固定 window，比 mean/median | 深度統計差異 |
| G2/G3 | 固定 2D 輸出、固定 statistic，比 7/11 window | 深度視窗差異 |
| G4/G5 | 固定點／depth／外形，比 refinement off/on | 局部修正價值 |
| G6/G7（選配） | 固定 grasp/release，以幾何中點或 VLM anchor 產路徑 | 多輸出一個 waypoint 是否有額外價值 |

B0 沒有語意 waypoint，該欄 N/A；若需比較 path validity，给所有組別同一 geometry-only routing 作 control，不把缺少 waypoint 算失敗。L2 多模型分工可有一個 Qwen＋兩個 Molmo requests，实际依實作計數，不宣稱一次 query。

單點 B0 問句若從複合指令改寫，規則事先凍結並存檔；不可用人工 target label 只幫助 B0。不能重用 B1 的已推論角色而不計算 B0 額外成本。

## 5. 指標與分母

| 指標 | 定義／報告方式 |
|---|---|
| schema success | 有合法 envelope 的 attempts/所有 attempts；首次與 retry 後分開 |
| actionable coverage | 有所需角色／可用點的 plans/所有有效任務 plans |
| grasp/release hit | 命中人工 region 的 plans/所有該類有效任務 plans；棄權及解析失敗計未命中 |
| negative false-positive | 對缺失／不可判斷任務仍給 actionable plan 的比例；negative 分開 |
| joint semantic hit | grasp 與 release 同時命中，避免兩個單點率掩蓋聯合失敗 |
| geometry pass rate | 完整代理幾何通過/所有要求 3D 的 plans；另報有完整输入子集的 conditional rate |
| unknown/failure | 按 depth、table、object extent、destination、path、frame 拆分 |
| consistency | 同景同方法三個獨立輸出的 UV std；有效少於兩次時 N/A；附有效次數与角色是否相同 |
| 3D error | 僅有獨立 3D truth 子集可報；不拿同一深度推導的 truth 證明深度正確 |
| latency | cold/warm、p50/p90、樣本數；cache hits／network errors另報 |
| cost | 真實 HTTP attempts、per-model requests、retries、tokens、cache hits |

主要統計單位是 scene，三次 repeats 不是三個新場景；同一方法 27 plans 仍只有九個 holdout layouts。小樣本優先展示逐景配對結果；若需不確定區間，依 scene cluster 重抽樣，不能把每一步當獨立試驗。

Dev 選 B1/B2 策略：先比較 joint hit 與負例誤報，再比較格式失敗／coverage；品質相同時選實際 calls 較少，仍同分再看延遲。三景 dev 不足以宣稱統計等效，holdout 仍同時報 B1/B2。

## 6. 預算與重現

Phase 2 第一輪三景 B0/B1/B2 是 15 次正常請求，另三負例最多 15 次，加最多 10 次格式 retry、最多 10 次 smoke/debug，起始工作預算 50 次；本階段總 hard cap 200。每次改 prompt 建新 version，不重置 phase ledger。

Phase 4 九景 × 三 repeats × (2+2+1) = **135** 正常 Gemini requests，另保留 27 次格式 retry，工作預算 **162**，hard cap **200**。LEGACY、額外否定集或新模型雲端比較不在這 135 次內，要另外列預算。Local L1 在 B1 prompt 下通常 54 requests、B2 下 27；L2 依實際模型分工另計。共享 GPU 一次一個 request，分小批有人看管。

Cache primary key 必含 actual image bytes hash、prompt text hash、backend/model revision、generation settings、schema/adapter version、stage、replicate ID。Stage B 另包含實際 Stage A plan，因此不同 A 不能誤用 B cache。manifest 記 split hash、source hashes、run IDs 與 request ledger ID。

擬定每個 run 產物：`manifest.json`、`config.json`、`requests.jsonl`、`raw/`、`plans_2d/`、`plans_3d/`、`overlays/`、`metrics_per_scene.csv`、`failures.jsonl`。正常、棄權與錯誤全部可追溯，run 目錄不可覆蓋。

## 7. Demo 驗收敘事

依序展示「輸入指令與同場景」→「baseline 與多點圖」→「深度表面與真正搬運高度」→「幾何驗證／修正」→「一次與兩次呼叫的全樣本比較」→「local 差異」→「一個失敗與下一步」。

2D 命中、3D observed validity、IK 成功、物理操作成功是四個不同指標；本週沒做真機試驗時，最後一項固定 NOT_EXECUTED。
