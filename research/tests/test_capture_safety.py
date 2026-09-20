from __future__ import annotations

import ast
import unittest
from pathlib import Path


class CaptureSafetyTest(unittest.TestCase):
    def test_no_motion_capable_ros_api(self) -> None:
        path = Path(__file__).resolve().parents[1] / "scripts" / "capture_scene.py"
        tree = ast.parse(path.read_text())
        forbidden = {"create_publisher", "create_client", "ActionClient"}
        seen = {
            node.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Attribute) and node.attr in forbidden
        }
        seen |= {
            node.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Name) and node.id in forbidden
        }
        self.assertEqual(seen, set())


if __name__ == "__main__":
    unittest.main()
