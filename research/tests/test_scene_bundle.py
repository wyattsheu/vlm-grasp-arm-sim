from __future__ import annotations

import json
import shutil
import sys
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mpg.scene_bundle import sha256_file, transform_matrix, validate_scene, write_json


class SceneBundleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.scene = ROOT / ".tmp" / "test_scene_bundle"
        shutil.rmtree(self.scene, ignore_errors=True)
        self.scene.mkdir(parents=True)
        Image.fromarray(np.zeros((3, 4, 3), dtype=np.uint8), mode="RGB").save(
            self.scene / "rgb.png"
        )
        Image.fromarray(np.ones((3, 4), dtype=np.uint16)).save(
            self.scene / "depth.png"
        )
        write_json(self.scene / "camera_info.json", {
            "width": 4, "height": 3, "k": [10, 0, 2, 0, 10, 1, 0, 0, 1],
            "depth_scale_m": 0.001,
        })
        write_json(self.scene / "tf.json", {
            "matrix": transform_matrix([0.1, 0.2, 0.3], [0, 0, 0, 1])
        })
        write_json(self.scene / "joint_states.json", {
            "names": ["joint1"], "positions": [0.0]
        })
        (self.scene / "instruction.txt").write_text("put cup in bowl\n")
        names = ["rgb.png", "depth.png", "camera_info.json", "tf.json",
                 "joint_states.json", "instruction.txt"]
        write_json(self.scene / "manifest.json", {
            "capture_complete": True,
            "sha256": {name: sha256_file(self.scene / name) for name in names},
        })

    def tearDown(self) -> None:
        shutil.rmtree(self.scene, ignore_errors=True)

    def test_valid_bundle(self) -> None:
        result = validate_scene(self.scene)
        self.assertEqual((result["width"], result["height"]), (4, 3))
        self.assertEqual(result["depth_dtype"], "uint16")
        self.assertAlmostEqual(result["rotation_determinant"], 1.0)

    def test_dimension_mismatch_fails(self) -> None:
        Image.fromarray(np.ones((2, 4), dtype=np.uint16)).save(
            self.scene / "depth.png"
        )
        with self.assertRaisesRegex(ValueError, "shape mismatch"):
            validate_scene(self.scene, verify_hashes=False)

    def test_missing_depth_scale_fails(self) -> None:
        data = json.loads((self.scene / "camera_info.json").read_text())
        del data["depth_scale_m"]
        write_json(self.scene / "camera_info.json", data)
        with self.assertRaisesRegex(ValueError, "depth_scale_m"):
            validate_scene(self.scene, verify_hashes=False)

    def test_hash_mismatch_fails(self) -> None:
        (self.scene / "instruction.txt").write_text("changed\n")
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            validate_scene(self.scene)

    def test_explicit_missing_tf_is_valid_for_2d(self) -> None:
        write_json(self.scene / "tf.json", {
            "status": "UNAVAILABLE", "matrix": None,
            "restriction": "2D_ONLY_UNTIL_EXTRINSIC_IS_RECONSTRUCTED",
        })
        manifest = json.loads((self.scene / "manifest.json").read_text())
        manifest["sha256"]["tf.json"] = sha256_file(self.scene / "tf.json")
        write_json(self.scene / "manifest.json", manifest)
        result = validate_scene(self.scene)
        self.assertEqual(result["tf_status"], "UNAVAILABLE")
        self.assertIsNone(result["rotation_determinant"])


if __name__ == "__main__":
    unittest.main()
