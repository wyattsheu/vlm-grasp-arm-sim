# Handoff validation

Validated on the source host on 2026-09-13:

- `tools/collect_server_info.sh` and `tools/verify_bundle.sh`: Bash syntax PASS.
- Packaged MPG tests: 108 tests PASS with `PYTHONDONTWRITEBYTECODE=1`.
- Search found no copied Piper SDK, CAN implementation, arm publisher/client, private-key marker, API-key assignment, `.env`, symlink, or remaining Python bytecode in the selected bundle after cleanup.
- `PIPER_LICENSE` is included.

Still NOT RUN:

- Archive extraction on the PRO 6000 server.
- Server compatibility checker and Isaac Sim installation.
- URDF import into Isaac Sim and generated USD validation.
- ROS 2, controller, MoveIt, VLM inference, simulated motion, Thor connection, IK, or physical execution.

The checksum verification result is generated after all bundle files are finalized. Run `bash tools/verify_bundle.sh` after transfer.

