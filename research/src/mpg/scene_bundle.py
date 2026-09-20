"""Pure, offline SceneBundle serialization and validation helpers."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


REQUIRED_FILES = (
    "rgb.png",
    "camera_info.json",
    "tf.json",
    "joint_states.json",
    "instruction.txt",
    "manifest.json",
)


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def transform_matrix(
    translation_xyz: list[float], quaternion_xyzw: list[float]
) -> list[list[float]]:
    """Return a homogeneous transform for a normalized xyzw quaternion."""
    if len(translation_xyz) != 3 or len(quaternion_xyzw) != 4:
        raise ValueError("translation must have 3 and quaternion must have 4 values")
    x, y, z, w = (float(v) for v in quaternion_xyzw)
    norm = float(np.linalg.norm([x, y, z, w]))
    if norm < 1e-12:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation_xyz, dtype=float)
    return matrix.tolist()


def _read_depth(scene_dir: Path) -> tuple[np.ndarray, str]:
    png = scene_dir / "depth.png"
    npy = scene_dir / "depth.npy"
    if png.exists() == npy.exists():
        raise ValueError("exactly one of depth.png or depth.npy must exist")
    if png.exists():
        array = np.asarray(Image.open(png))
        # Pillow commonly exposes a 16-bit grayscale PNG as mode "I" /
        # int32. Preserve the file's unsigned 16-bit contract by checking the
        # value range before converting the in-memory representation.
        if array.dtype == np.int32 and array.size and array.min() >= 0 and array.max() <= 65535:
            array = array.astype(np.uint16)
        if array.dtype != np.uint16:
            raise ValueError(f"depth.png must be uint16, got {array.dtype}")
        return array, "depth.png"
    array = np.load(npy, allow_pickle=False)
    if array.dtype not in (np.float32, np.float64):
        raise ValueError(f"depth.npy must be float32/float64, got {array.dtype}")
    return array, "depth.npy"


def validate_scene(scene_dir: Path, *, verify_hashes: bool = True) -> dict[str, Any]:
    """Validate one completed SceneBundle and return measurable properties."""
    scene_dir = Path(scene_dir)
    missing = [name for name in REQUIRED_FILES if not (scene_dir / name).is_file()]
    if missing:
        raise ValueError(f"missing required files: {missing}")

    rgb = np.asarray(Image.open(scene_dir / "rgb.png").convert("RGB"))
    depth, depth_name = _read_depth(scene_dir)
    if rgb.shape[:2] != depth.shape[:2]:
        raise ValueError(f"RGB/depth shape mismatch: {rgb.shape[:2]} vs {depth.shape[:2]}")

    camera = json.loads((scene_dir / "camera_info.json").read_text())
    height, width = rgb.shape[:2]
    if (camera.get("width"), camera.get("height")) != (width, height):
        raise ValueError("CameraInfo dimensions do not match captured images")
    k = camera.get("k")
    if not isinstance(k, list) or len(k) != 9 or float(k[0]) <= 0 or float(k[4]) <= 0:
        raise ValueError("CameraInfo K must contain 9 values with positive fx/fy")
    scale = camera.get("depth_scale_m")
    if not isinstance(scale, (int, float)) or not np.isfinite(scale) or scale <= 0:
        raise ValueError("depth_scale_m must be explicit, finite, and positive")

    transform = json.loads((scene_dir / "tf.json").read_text())
    tf_status = transform.get(
        "status", "AVAILABLE" if transform.get("matrix") is not None else "UNAVAILABLE"
    )
    if tf_status == "AVAILABLE":
        matrix = np.asarray(transform.get("matrix"), dtype=float)
        if matrix.shape != (4, 4) or not np.allclose(
            matrix[3], [0, 0, 0, 1], atol=1e-7
        ):
            raise ValueError("T_base_camera must be a homogeneous 4x4 matrix")
        rotation = matrix[:3, :3]
        orthogonal_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
        determinant = float(np.linalg.det(rotation))
        if orthogonal_error > 1e-5 or abs(determinant - 1.0) > 1e-5:
            raise ValueError("T_base_camera rotation is not a valid SO(3) matrix")
    elif tf_status == "UNAVAILABLE" and transform.get("matrix") is None:
        orthogonal_error = None
        determinant = None
    else:
        raise ValueError("tf status must be AVAILABLE with a matrix or UNAVAILABLE")

    joints = json.loads((scene_dir / "joint_states.json").read_text())
    if not joints.get("names") or len(joints["names"]) != len(joints.get("positions", [])):
        raise ValueError("joint names and positions must be non-empty and have equal length")
    if not (scene_dir / "instruction.txt").read_text(encoding="utf-8").strip():
        raise ValueError("instruction.txt is empty")

    manifest = json.loads((scene_dir / "manifest.json").read_text())
    if not manifest.get("capture_complete"):
        raise ValueError("manifest capture_complete is not true")
    if verify_hashes:
        expected = manifest.get("sha256", {})
        hashed_names = ["rgb.png", depth_name, "camera_info.json", "tf.json",
                        "joint_states.json", "instruction.txt"]
        for name in hashed_names:
            if expected.get(name) != sha256_file(scene_dir / name):
                raise ValueError(f"SHA-256 mismatch for {name}")

    finite = np.isfinite(depth)
    nonzero = finite & (depth > 0)
    return {
        "width": width,
        "height": height,
        "depth_file": depth_name,
        "depth_dtype": str(depth.dtype),
        "valid_depth_fraction": float(nonzero.mean()),
        "tf_status": tf_status,
        "rotation_orthogonal_error": orthogonal_error,
        "rotation_determinant": determinant,
    }
