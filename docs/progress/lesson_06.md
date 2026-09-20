# Lesson 6 — 夾爪、碰撞與接觸

狀態：PASS（simulation physics acceptance）

## 概念與架構

真正的 physics grasp 必須靠兩側 contact force 與摩擦托住動態物體。若每幀直接把物體 pose 寫到夾爪附近，只能驗證動畫流程，不能算抓取成功。

## Done／Verified

- `joint7`／`joint8` 對稱開合，行程為 `+0.035/-0.035 m`；`joint8` 是被動 mimic。
- 紅色方塊從建立起就是 dynamic rigid body，質量 `0.08 kg`；frame 105 後開啟重力。
- 抓住後沒有 pose attachment，也沒有 kinematic 切換。
- 最大指尖接觸力：link7 `10.0609 N`、link8 `10.0553 N`。
- payload 高度由 `0.45037 m` 抬到 `0.60614 m`，開爪後落到地面 `0.01750 m`。
- 雙側接觸、抬升、放開後落下、有限數值全部 PASS。
- 報告：`out/lesson_06/physics_grasp/physics_grasp_report.json`。
- 影片：`out/lesson_06/physics_grasp/robot129_physics_grasp.mp4`。

## NOT RUN／UNVERIFIED

- 靜摩擦 4.0、動摩擦 3.0、stiffness/damping 為讓模擬穩定的估計，不代表 Piper 原廠或 Robot 129 實測值。
- 尚未建立真實指尖材質與 payload mass/inertia 的量測資料庫。

## 下一課

輸出同步 RGB-D、CameraInfo、TF、JointState 與 SceneBundle。
