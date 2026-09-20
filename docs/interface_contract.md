# Plan3D 介面草案 v0.1

DESIGN ONLY，尚未實作。預先寫清楚交接語意；Phase 5 的 IK／整合仍需核准。

外層必備：`schema_version`、`scene_id`、`instruction`、`image_sha256`、`capture_time`、`camera_frame`、`base_frame`、`transform_time`、`calibration_id`、`backend`、`prompt_hash`、`replicate_id`、`status`、`failure_reasons`、`steps`、`metrics`。

每步必備：`step_id`、`type`、`desc`、`geometric_meaning`、`reference_kind`、`point_yx_norm1000`（可 null）、`point_status`、`surface_position_base_m`（可 null）、`object_reference_position_base_m`（可 null）、`tcp_pose_base`（可 null）、`depth_quality`、`refinement`。點不可用時必须有原因，不能用零向量填補。

| 欄位 | 語意 |
|---|---|
| `reference_kind` | grasp contact / object bottom center / destination support，不能互換 |
| `point_yx_norm1000` | 原始模型圖上的 [y,x]；另存 crop/resize mapping |
| `surface_position_base_m` | 深度反投影的觀測表面 |
| `object_reference_position_base_m` | 幾何建議的搬運物體參考點 |
| `tcp_pose_base` | 有已確認 grasp transform／orientation 才能生成；米＋`quaternion_xyzw` |
| `geometry_status` | PASS_OBSERVED / FAIL / UNKNOWN，附外形、解析度、margin |
| `ik_status` | NOT_RUN / PASS / FAIL，不能由 workspace bounds 代替 |
| `robot_collision_status` | NOT_RUN / PASS / FAIL，與 object-point-cloud 檢查不同 |
| `execution_status` | 此專案一律 NOT_EXECUTED |

`transport_path` 為獨立有序陣列：lift、transit(s)、preplace、descent；必須指定其點表示 object reference 或 TCP。三個 semantic steps 不保證只需三個運動點。記錄每段的未知空間、觀測 clearance 與失敗原因。

`metrics` 同時記 logical plan 數、各模型實際 request 數、retries、cache hits、tokens、network latency。不要把一次上層函式呼叫視為一次 VLM 請求。

Tuan 需要確認：實際 base/TCP frame 名、pose 四元數順序、gripper width 單位、grasp-to-object offset、orientation policy、控制器如何插補 lift/transit/descent、是否已有全機身碰撞檢查。未確認前，不輸出看似可執行的預設 pose。

交接只寫檔案。沒有 ROS publisher、service/action client、運動呼叫；讀到 JSON 不等於授權執行。底盤移動、物體移動或 calibration 變更後，旧 snapshot 的 plan 失效，需重新觀測。
