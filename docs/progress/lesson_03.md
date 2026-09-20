# Lesson 3 — Piper URDF 與 TF tree

狀態：PASS（硬體 revision UNVERIFIED）

## 概念與架構

URDF 定義 link、joint、慣性、visual 與 collision；TF tree 則是這些 frame 在某個關節狀態下的空間關係。來源位於唯讀的 `robot/vendor`，衍生驗證由 `tools/build_robot129_description.py` 執行。

## Done／Verified

- 唯讀解析官方 Piper URDF，建立 joint 名稱、parent/child、axis、limit 與單位核對資料。
- 原始手臂包含 6 個 revolute arm joints 與 2 個 prismatic finger joints。
- 20 個 visual/collision mesh references 全部存在且非空。
- 衍生 Robot 129 tree 有 17 links、16 joints，唯一 root 為 `world`。
- 產物：`out/lesson_03/urdf_validation.json`，狀態 PASS。

## NOT RUN／UNVERIFIED

- 尚未與 Robot 129 銘牌、硬體 revision、實機零位與 joint 正方向比對。
- 所有判斷只適用於目前 bundle 內的官方 3D/URDF 來源。

## 下一課

建立新的 `robot129_description`，不修改 vendor。
