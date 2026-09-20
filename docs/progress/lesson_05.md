# Lesson 5 — Piper USD 與 FK

狀態：PASS

## 概念與架構

URDF 是機器人結構交換格式；USD 保存 Isaac stage、physics schema、articulation 與 drive 設定。Importer 腳本是可重建 USD 的來源，USD 本身是產物。

## Done／Verified

- Isaac Sim 6.0.1 importer 由 `tools/import_robot129_usd.sh` 產生 `sim/assets/robot129/robot129/robot129.usda`。
- 匯入結果 69 prims、6 revolute、2 prismatic joints，fixed base；PhysX 可建立 articulation。
- importer settings、來源 URDF 與 USD 路徑寫入 `sim/assets/robot129/import_report.json`。
- 用相同關節狀態比較 URDF FK 與 Isaac `gripper_base` pose：位置誤差 `1.58e-7 m`，角度誤差 `0 rad`。
- FK 產物：`out/lesson_05/fk_consistency.json`，狀態 PASS。

## NOT RUN／UNVERIFIED

- 尚未用實機量測驗證絕對座標與 link 製造誤差。
- drive gains 是 simulation estimate。

## 下一課

驗證兩指耦合、碰撞、接觸、摩擦與動態 payload。
