from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "capture_scene", ROOT / "scripts" / "capture_scene.py"
)
capture_scene = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(capture_scene)


class CaptureDecodingTest(unittest.TestCase):
    def test_bgr_with_row_padding_becomes_rgb(self) -> None:
        message = SimpleNamespace(
            encoding="bgr8", height=1, width=2, step=8,
            data=bytes([3, 2, 1, 6, 5, 4, 99, 99]),
        )
        decoded = capture_scene.decode_color(message)
        np.testing.assert_array_equal(
            decoded, np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        )

    def test_big_endian_uint16_depth(self) -> None:
        message = SimpleNamespace(
            encoding="16UC1", height=1, width=2, step=4, is_bigendian=True,
            data=bytes([0x01, 0x02, 0x03, 0x04]),
        )
        decoded = capture_scene.decode_depth(message)
        self.assertEqual(decoded.dtype, np.uint16)
        np.testing.assert_array_equal(decoded, np.array([[0x0102, 0x0304]], np.uint16))


if __name__ == "__main__":
    unittest.main()
