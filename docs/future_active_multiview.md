# Future：wrist-camera active multi-view

DESIGN ONLY。本週不實作，也不控制手臂改變視角。

Wyatt/Tuan 人工選取 2–3 個可觀測相同靜止物體的 wrist poses；逐個穩定後擷取 RGB-D、相機 K、曝光時間及當時關節。保持底盤固定；若底盤移動，需額外已驗證的共同 world transform。第 i 視角採 `T_base_camera_i = T_base_ee(q_i) * T_ee_camera`，每個矩陣記 frame、方向與 calibration hash。

各視角先確認同一語意 reference，再比較：（a）單視角 RGB-D；（b）多視角 depth points 融合；（c）三角化；（d）三角化＋reference-ray voting。固定模型／prompt，以免把新增視角收益與模型變化混在一起。

ZeroDex 附錄參數可作研究起始參考：640×480 下 20 px reprojection threshold，約半數視角 consensus，以及 ray depth 0.5–2.0 m、步長 0.05 m；不能直接套到 wrist camera 的近距離工作範圍。應依相機解析度、實測可用深度、像素定位噪音與 baseline 設定，並事先凍結評估配置。[本機閱讀筆記](../refs/reading_notes.md)

只有兩個視角時，一對點自行三角化後的低重投影誤差不構成獨立的跨視角驗證。至少檢查正深度、視線夾角、depth consistency 與不確定度；第三視角可用於留一驗證。退化、跨視角誤認、遮蔽不一致時棄權。

風險與觀測指標：baseline 太短→深度不確定度高；手眼誤差→多視角系統偏差；曝光時刻姿態錯誤→重投影誤差；移動模糊→2D 定位不穩；移動物體→不符合靜態場景假設。先以校正板／已知 3D 點驗證外參與重投影，再研究 VLM。使用人為選姿態也應計入 capture 時間，不能只比較推論延遲。
