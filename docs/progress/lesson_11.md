# Lesson 11 — Thor adapter 與 sim-to-real rehearsal

狀態：PASS（server-local adapter rehearsal）；Thor-connected NOT RUN

## 概念與架構

部署 adapter 必須拒絕 stale observation、重複 sequence、錯誤 domain 與硬體 backend，讓 Thor 端只能對隔離的 Isaac 模擬器發命令。此層是部署介面驗收，不是實機授權。

## Done／Verified

- 建立 `deployment/robot129_adapter.py`、`deployment/verify_thor_loopback.py` 與操作說明。
- 使用 `127.0.0.1` ephemeral UDP 完成 20 次 observation roundtrip；median `0.0276 ms`、p95 `0.0505 ms`。
- 合法 simulation command 接受；stale observation、重複 sequence、hardware backend 與 production domain 全部拒絕。
- `thor_connected=false`、`hardware_connected=false`、硬體命令 0。
- 報告：`out/lesson_11/thor_adapter_loopback.json`。

## NOT RUN／UNVERIFIED

- 未取得 Thor host、測試網路或 aarch64 runtime，因此沒有跨主機 DDS/network latency、Thor GPU/memory 或 ARM rebuild 數據。
- CAN、Piper SDK、RealSense USB 與 production ROS domain 完全未連接。

## 下一步

若要做 Thor-connected rehearsal，需先確定 Thor 位址、隔離網路/domain、ARM 環境與回復方式；仍只能連 simulation adapter。
