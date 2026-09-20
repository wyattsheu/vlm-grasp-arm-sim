# Lesson 9 — MoveIt 2 與 MoveIt Task Constructor

狀態：PASS（plan-only simulation baseline）

## 概念與架構

MoveIt 將 URDF/SRDF、kinematics、joint limits、controller mapping 與 PlanningScene 組成 motion planning；MTC 再把任務拆成可診斷的 stages。

## Done／Verified

- user-space MoveIt 2.12.4、OMPL、Pilz 與 `move_group` 已建立；`robot129_moveit_config` package build PASS。
- 固定目標加入 `side_obstacle` collision object，OMPL `RRTConnect` 產生 21 點 trajectory，規劃時間 `0.1190 s`，終點誤差 `0.000749 rad`，MoveIt success code 1。
- MTC 官方 `ros2-0.1.4` C++ core 由固定 commit 建置；來源與 patch 記在 `ros2_ws/src/external/README.md`。
- MTC 產生 1 個解，stage graph 為 CURRENT_STATE、APPROACH、GRASP、LIFT、TRANSIT、PREPLACE、RELEASE、RETREAT。
- 報告：`out/lesson_09/moveit_fixed_plan.json`、`out/lesson_09/mtc_plan.json`。
- 全程 `hardware_commands=0`，launch 結束後沒有殘留 `move_group`。

## NOT RUN／UNVERIFIED

- 尚未把整條 MTC solution 交給 Thor 或真實控制器執行。
- 真實腕上相機與 payload 幾何尚未校正，不能把此 collision result 外推到實機安全性。

## 下一課

用實際 Isaac SceneBundle 執行 deterministic MPG replay，並和 PhysX 成功條件分開記錄。
