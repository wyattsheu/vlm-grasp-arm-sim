#!/usr/bin/env python3
"""Inspect the saved Robot 129 Isaac RGB-D SceneBundle without starting Isaac."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = ROOT / "out/integrated_demo/scene_bundle"
DEFAULT_OUTPUT = ROOT / "out/camera_inspection"


def colorize_depth(depth: np.ndarray) -> tuple[np.ndarray, float, float]:
    valid = np.isfinite(depth) & (depth > 0)
    if not valid.any():
        raise ValueError("depth has no finite positive pixels")
    near, far = np.percentile(depth[valid], [2.0, 98.0])
    if far <= near:
        far = near + 1e-6
    x = np.clip((depth - near) / (far - near), 0.0, 1.0)
    red = np.clip(2.0 * x, 0.0, 1.0)
    green = np.clip(2.0 - np.abs(4.0 * x - 2.0), 0.0, 1.0)
    blue = np.clip(2.0 * (1.0 - x), 0.0, 1.0)
    image = (np.stack([red, green, blue], axis=-1) * 255).astype(np.uint8)
    image[~valid] = (25, 25, 25)
    return image, float(near), float(far)


def font(size: int):
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--u", type=int, default=320, help="pixel column")
    parser.add_argument("--v", type=int, default=240, help="pixel row")
    args = parser.parse_args()

    rgb = np.asarray(Image.open(args.scene / "rgb.png").convert("RGB"))
    depth = np.squeeze(np.load(args.scene / "depth.npy"))
    if depth.ndim != 2:
        raise SystemExit(f"FAIL: expected 2-D depth, got {depth.shape}")
    camera_info = json.loads((args.scene / "camera_info.json").read_text())
    tf = json.loads((args.scene / "tf.json").read_text())
    manifest_path = args.scene / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}

    if rgb.shape[:2] != depth.shape:
        raise SystemExit(f"FAIL: RGB {rgb.shape[:2]} and depth {depth.shape} are not aligned")
    height, width = depth.shape
    if not (0 <= args.u < width and 0 <= args.v < height):
        raise SystemExit(f"FAIL: pixel ({args.u}, {args.v}) outside {width}x{height}")

    valid = np.isfinite(depth) & (depth > 0)
    depth_rgb, near, far = colorize_depth(depth)
    d = float(depth[args.v, args.u])
    k = camera_info["k"]
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    point = None
    if np.isfinite(d) and d > 0:
        point = {
            "x_m": (args.u - cx) / fx * d,
            "y_m": (args.v - cy) / fy * d,
            "z_m": d,
        }

    args.output.mkdir(parents=True, exist_ok=True)
    Image.fromarray(depth_rgb).save(args.output / "depth_color.png")

    header = 94
    canvas = Image.new("RGB", (width * 2, height + header), "#20242a")
    canvas.paste(Image.fromarray(rgb), (0, header))
    canvas.paste(Image.fromarray(depth_rgb), (width, header))
    draw = ImageDraw.Draw(canvas)
    draw.text((16, 10), "Isaac RGB", font=font(24), fill="white")
    draw.text((width + 16, 10), "Depth to optical image plane", font=font(24), fill="white")
    draw.text((16, 48), f"{width}x{height} selected pixel=({args.u},{args.v})", font=font(18), fill="#a9d6ff")
    draw.text((width + 16, 48), f"range {near:.3f}m to {far:.3f}m; pixel={d:.3f}m", font=font(18), fill="#a9d6ff")
    radius = 6
    for offset_x in (0, width):
        draw.ellipse(
            (offset_x + args.u - radius, header + args.v - radius,
             offset_x + args.u + radius, header + args.v + radius),
            outline="white", width=2,
        )
    preview = args.output / "rgbd_preview.png"
    canvas.save(preview)

    report = {
        "status": "PASS",
        "simulation_only": True,
        "camera_prim": tf.get("camera_prim", "/World/RecordingCamera"),
        "camera_role": "wrist_rgbd" if tf.get("attached_to") else "fixed_demo_recording_camera",
        "wrist_camera_live_ros": bool(tf.get("attached_to")),
        "rgb": {"path": str(args.scene / "rgb.png"), "shape": list(rgb.shape), "dtype": str(rgb.dtype)},
        "depth": {
            "path": str(args.scene / "depth.npy"),
            "shape": list(depth.shape),
            "dtype": str(depth.dtype),
            "units": camera_info.get("depth_units"),
            "definition": camera_info.get("depth_definition"),
            "valid_fraction": float(valid.mean()),
            "min_m": float(depth[valid].min()),
            "median_m": float(np.median(depth[valid])),
            "max_m": float(depth[valid].max()),
            "color_range_2_98_percentile_m": [near, far],
        },
        "alignment": {"same_pixel_grid": rgb.shape[:2] == depth.shape, "width": width, "height": height},
        "selected_pixel": {"u": args.u, "v": args.v, "depth_m": d, "camera_point": point},
        "camera_info": camera_info,
        "tf": tf,
        "manifest_status": manifest.get("status"),
        "preview": str(preview),
    }
    report_path = args.output / "rgbd_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
