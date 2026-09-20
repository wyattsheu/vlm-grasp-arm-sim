# Robot 129 SceneBundle data

Every scene lives at `data/scenes/<scene_id>/`. Captures are immutable: the capture tool refuses to overwrite a scene and removes an incomplete staging directory on failure.

Required capture files:

- `rgb.png`
- exactly one of `depth.png` (uint16 raw units) or `depth.npy` (float depth)
- `camera_info.json`, including explicit `depth_scale_m`
- `tf.json`, containing `T_base_camera` at the RGB timestamp
- `joint_states.json`
- `instruction.txt`
- `manifest.json`, including timestamps, deltas and SHA-256 hashes

`labels.json` is added after capture. Bounding boxes use inclusive pixel `[xmin,ymin,xmax,ymax]`.

Planned IDs are S1_01–S1_04 (simple), S2_01–S2_04 (obstacle), S3_01–S3_04 (clutter/ambiguity), and N_01–N_03 (negative diagnostics). Use a fixed base with target and destination visible in the same frame for the first dataset.

The root disk had only 42 GiB free at Phase 0. Store one snapshot per scene; do not record bags or videos here. Captured data count is currently zero.

Do not capture formal scenes in a dark, unattended lab. A D435 depth stream may
still return infrared-derived measurements, but this project's visual
grounding depends on a usable visible-light RGB frame. Connection-test frames
must not be assigned an S1/S2/S3 scene ID.

Run capture only from a sourced ROS 2 environment that can see this workspace at
the same absolute path, after the camera, base frame, and depth scale are
confirmed:

```bash
python3 scripts/capture_scene.py \
  --scene-id S1_01 \
  --instruction "put the cup in the bowl" \
  --base-frame base_link \
  --allow-missing-tf \
  --depth-scale-m CONFIRMED_METERS_PER_RAW_UNIT

python3 scripts/validate_scene.py data/scenes/S1_01
python3 scripts/label_scene.py data/scenes/S1_01 --obstacles 0 --annotator Wyatt
```

The capture command has not been run. Do not substitute guessed values for the
`CONFIRMED_METERS_PER_RAW_UNIT`. The Piper URDF names the arm frame
`base_link`, but it was absent from the live TF graph. With
`--allow-missing-tf`, the bundle records `status=UNAVAILABLE` and remains
2D-only until FK plus hand-eye reconstruction is reviewed.
