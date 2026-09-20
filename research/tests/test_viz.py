from __future__ import annotations

import json
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from PIL import Image  # noqa: E402

from mpg.schema import (  # noqa: E402
    Meta,
    StageAPlan,
    merge_plan,
    parse_stage_b_response,
    pixel_from_norm1000,
)
import numpy as np  # noqa: E402

from mpg.schema import AffordanceRegion, PointStatus  # noqa: E402
from mpg.viz import (  # noqa: E402
    render_affordance_overlay,
    render_baseline_overlay,
    render_candidate_ghosts,
    render_plan_overlay,
    side_by_side,
)


def _plan(point_status: str = "localized"):
    stage_a_json = {
        "schema_version": "0",
        "view_description": "d",
        "mode": "pick",
        "target": {"name": "cup", "visibility": "visible"},
        "destination": {"name": "bowl", "visibility": "visible"},
        "tool": None,
        "status": "ready",
        "reason_codes": [],
        "steps": [
            {"step_id": "s0", "type": "GRASP", "desc": "d", "reference_kind": "grasp_contact", "geometric_meaning": "g"},
            {"step_id": "s1", "type": "WAYPOINT", "desc": "d", "reference_kind": "transit_anchor", "geometric_meaning": "g"},
            {"step_id": "s2", "type": "RELEASE", "desc": "d", "reference_kind": "destination_support", "geometric_meaning": "g"},
        ],
    }
    stage_a = StageAPlan.from_json_text(json.dumps(stage_a_json))
    if point_status == "localized":
        steps_b = [
            {"step_id": "s0", "point_yx_norm1000": [500, 500], "point_status": "localized", "reason_codes": []},
            {"step_id": "s1", "point_yx_norm1000": [200, 300], "point_status": "localized", "reason_codes": []},
            {"step_id": "s2", "point_yx_norm1000": [800, 700], "point_status": "localized", "reason_codes": []},
        ]
    else:
        steps_b = [
            {"step_id": "s0", "point_yx_norm1000": [500, 500], "point_status": "localized", "reason_codes": []},
            {"step_id": "s1", "point_yx_norm1000": None, "point_status": "unavailable", "reason_codes": ["OCCLUDED"]},
            {"step_id": "s2", "point_yx_norm1000": [800, 700], "point_status": "localized", "reason_codes": []},
        ]
    located = parse_stage_b_response(json.dumps(steps_b), stage_a)
    return merge_plan(
        scene_id="t", instruction="i", backend="fake", stage_a_plan=stage_a,
        located_steps=located, meta=Meta(latency_s=0.0, n_calls=2, parse_retries=0),
    )


class RenderPlanOverlayTest(unittest.TestCase):
    def test_output_is_wider_or_equal_and_taller_than_input(self) -> None:
        image = Image.new("RGB", (200, 100), color=(255, 255, 255))
        out = render_plan_overlay(image, _plan())
        self.assertEqual(out.width, 200)
        self.assertGreater(out.height, 100)

    def test_grasp_point_pixel_is_drawn_near_expected_location(self) -> None:
        image = Image.new("RGB", (200, 100), color=(255, 255, 255))
        plan = _plan()
        out = render_plan_overlay(image, plan)
        grasp_step = plan.steps[0]
        x_px, y_px = pixel_from_norm1000(grasp_step.point_yx_norm1000, width=200, height=100)
        # the ellipse outline should place a non-white (red-ish) pixel on
        # the circle boundary a few px to the right of center
        sample = out.getpixel((int(x_px) + 8, int(y_px)))
        self.assertNotEqual(sample, (255, 255, 255))

    def test_unavailable_step_grows_the_status_strip(self) -> None:
        image = Image.new("RGB", (200, 100), color=(255, 255, 255))
        out_ok = render_plan_overlay(image, _plan("localized"))
        out_partial = render_plan_overlay(image, _plan("unavailable"))
        self.assertGreater(out_partial.height, out_ok.height)


def _tool_plan():
    stage_a_json = {
        "schema_version": "0",
        "view_description": "d",
        "mode": "tool",
        "target": {"name": "apple", "visibility": "visible"},
        "destination": {"name": "mat", "visibility": "visible"},
        "tool": {"name": "brush", "visibility": "visible"},
        "status": "ready",
        "reason_codes": [],
        "steps": [
            {"step_id": "s0", "type": "GRASP", "desc": "d", "reference_kind": "grasp_contact", "geometric_meaning": "g"},
            {"step_id": "s0b", "type": "FUNCTIONAL_TIP", "desc": "d", "reference_kind": "tool_functional_tip", "geometric_meaning": "g"},
            {"step_id": "s1", "type": "APPLY_ACTION", "desc": "d", "reference_kind": "transit_anchor", "geometric_meaning": "g"},
            {"step_id": "s2", "type": "RELEASE", "desc": "d", "reference_kind": "destination_support", "geometric_meaning": "g"},
        ],
    }
    stage_a = StageAPlan.from_json_text(json.dumps(stage_a_json))
    # Spread well apart on the small 200x100 test canvas so neighboring
    # markers/labels don't bleed into each other's sampled pixel.
    steps_b = [
        {"step_id": "s0", "point_yx_norm1000": [100, 100], "point_status": "localized", "reason_codes": []},
        {"step_id": "s0b", "point_yx_norm1000": [100, 900], "point_status": "localized", "reason_codes": []},
        {"step_id": "s1", "point_yx_norm1000": [900, 100], "point_status": "localized", "reason_codes": []},
        {"step_id": "s2", "point_yx_norm1000": [900, 900], "point_status": "localized", "reason_codes": []},
    ]
    located = parse_stage_b_response(json.dumps(steps_b), stage_a)
    return merge_plan(
        scene_id="t", instruction="i", backend="fake", stage_a_plan=stage_a,
        located_steps=located, meta=Meta(latency_s=0.0, n_calls=2, parse_retries=0),
    )


class RenderPlanOverlayToolModeTest(unittest.TestCase):
    def test_functional_tip_point_is_drawn(self) -> None:
        image = Image.new("RGB", (200, 100), color=(255, 255, 255))
        plan = _tool_plan()
        out = render_plan_overlay(image, plan)
        tip_step = plan.steps[1]
        self.assertEqual(tip_step.spec.type.value, "FUNCTIONAL_TIP")
        x_px, y_px = pixel_from_norm1000(tip_step.point_yx_norm1000, width=200, height=100)
        sample = out.getpixel((int(x_px) + 8, int(y_px)))
        self.assertNotEqual(sample, (255, 255, 255))

    def test_four_points_render_at_four_distinct_colors(self) -> None:
        image = Image.new("RGB", (200, 100), color=(255, 255, 255))
        plan = _tool_plan()
        out = render_plan_overlay(image, plan)
        colors = set()
        for step in plan.steps:
            x_px, y_px = pixel_from_norm1000(step.point_yx_norm1000, width=200, height=100)
            colors.add(out.getpixel((int(x_px) + 8, int(y_px))))
        self.assertEqual(len(colors), 4)  # GRASP/FUNCTIONAL_TIP/APPLY_ACTION/RELEASE all distinct


class RenderBaselineOverlayTest(unittest.TestCase):
    def test_both_missing_does_not_crash_and_adds_status_text(self) -> None:
        image = Image.new("RGB", (100, 50), color=(255, 255, 255))
        out = render_baseline_overlay(
            image, target_point_px=None, destination_point_px=None
        )
        self.assertGreater(out.height, image.height)

    def test_both_present_draws_two_distinct_colors(self) -> None:
        image = Image.new("RGB", (100, 50), color=(255, 255, 255))
        out = render_baseline_overlay(
            image, target_point_px=(20, 20), destination_point_px=(80, 30)
        )
        p1 = out.getpixel((28, 20))
        p2 = out.getpixel((88, 30))
        self.assertNotEqual(p1, (255, 255, 255))
        self.assertNotEqual(p2, (255, 255, 255))
        self.assertNotEqual(p1, p2)  # grasp=red vs release=green


class SideBySideTest(unittest.TestCase):
    def test_dimensions(self) -> None:
        left = Image.new("RGB", (50, 40))
        right = Image.new("RGB", (60, 30))
        out = side_by_side(left, right, gap=10)
        self.assertEqual(out.width, 50 + 10 + 60)
        self.assertGreater(out.height, 40)  # label band height is now adaptive to image size


class RenderAffordanceOverlayTest(unittest.TestCase):
    def test_localized_draws_box_and_caption(self) -> None:
        image = Image.new("RGB", (100, 100), color=(120, 120, 120))
        region = AffordanceRegion(
            status=PointStatus.LOCALIZED, description="top face",
            point_yx_norm1000=(500.0, 500.0),
            bbox_yx_norm1000=(400.0, 400.0, 600.0, 600.0), reason_codes=[],
        )
        out = render_affordance_overlay(image, region, caption_lines=("time: t0", "object: cube"))
        self.assertEqual(out.width, image.width)
        self.assertGreater(out.height, image.height)  # caption strip appended
        # the box's outline color (orange) must appear somewhere in the image
        colors = {out.getpixel((x, 40)) for x in range(image.width)}
        self.assertIn((230, 140, 20), colors)

    def test_unavailable_draws_no_box_but_reports_reason_in_caption(self) -> None:
        image = Image.new("RGB", (100, 100), color=(120, 120, 120))
        region = AffordanceRegion(
            status=PointStatus.UNAVAILABLE, description=None,
            point_yx_norm1000=None, bbox_yx_norm1000=None,
            reason_codes=["TARGET_NOT_VISIBLE"],
        )
        out = render_affordance_overlay(image, region)
        self.assertGreater(out.height, image.height)  # UNAVAILABLE line still adds a strip
        top = out.crop((0, 0, image.width, image.height))
        self.assertEqual(list(top.getdata()), list(image.getdata()))  # image itself untouched


class RenderCandidateGhostsTest(unittest.TestCase):
    def _candidate(self, cid: str, *, score: float, accepted: bool = True, yaw: float = 0.0) -> dict:
        return {
            "candidate_id": cid,
            "tcp_pose": {"position_m": [0.0, 0.0, 1.0]},  # 1m in front of the camera
            "yaw_rad": yaw,
            "opening_width_m": 0.01,
            "score": score,
            "accepted": accepted,
            "rejection_reasons": [] if accepted else ["WIDTH_EXCEEDED"],
        }

    def test_chosen_and_rejected_candidates_render_distinct_colors(self) -> None:
        image = Image.new("RGB", (200, 200), color=(120, 120, 120))
        k_matrix = [100.0, 0.0, 100.0, 0.0, 100.0, 100.0, 0.0, 0.0, 1.0]
        t_camera_world = np.eye(4)  # world frame == camera frame for this synthetic test
        candidates = [
            self._candidate("G0", score=0.5, accepted=True),
            self._candidate("G1", score=0.2, accepted=False),
        ]
        out = render_candidate_ghosts(
            image, candidates, k_matrix=k_matrix, t_camera_world=t_camera_world,
            chosen_id="G0",
        )
        self.assertGreater(out.height, image.height)  # detail-line caption strip appended
        top = out.crop((0, 0, image.width, image.height))
        colors = set(top.getdata())
        self.assertIn((30, 200, 60), colors)   # chosen -> solid green
        self.assertIn((220, 40, 40), colors)   # rejected -> dashed red

    def test_candidate_behind_camera_is_skipped_not_misdrawn(self) -> None:
        image = Image.new("RGB", (50, 50), color=(0, 0, 0))
        k_matrix = [50.0, 0.0, 25.0, 0.0, 50.0, 25.0, 0.0, 0.0, 1.0]
        t_camera_world = np.eye(4)
        behind = self._candidate("G_BEHIND", score=0.1)
        behind["tcp_pose"]["position_m"] = [0.0, 0.0, -1.0]  # behind the camera plane
        out = render_candidate_ghosts(
            image, [behind], k_matrix=k_matrix, t_camera_world=t_camera_world,
        )
        top = out.crop((0, 0, image.width, image.height))
        self.assertEqual(list(top.getdata()), list(image.getdata()))  # nothing drawn


if __name__ == "__main__":
    unittest.main()
