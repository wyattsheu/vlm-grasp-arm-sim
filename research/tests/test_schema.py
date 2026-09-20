from __future__ import annotations

import json
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mpg.schema import (  # noqa: E402
    AffordanceRegion,
    LocatedStep,
    Meta,
    ParseError,
    PointStatus,
    ScenarioPlan,
    StageAPlan,
    merge_plan,
    parse_grasp_affordance_response,
    parse_single_call_response,
    parse_stage_0_response,
    parse_stage_b_response,
    pixel_from_norm1000,
    strip_code_fence,
)


def _stage_a_json(**overrides) -> dict:
    base = {
        "schema_version": "0",
        "view_description": "A white cup sits on a table next to a blue bowl.",
        "mode": "pick",
        "target": {"name": "white cup", "visibility": "visible"},
        "destination": {"name": "blue bowl", "visibility": "visible"},
        "tool": None,
        "status": "ready",
        "reason_codes": [],
        "steps": [
            {
                "step_id": "s0",
                "type": "GRASP",
                "desc": "cup body",
                "reference_kind": "grasp_contact",
                "geometric_meaning": "the cup's handle-side wall",
            },
            {
                "step_id": "s1",
                "type": "WAYPOINT",
                "desc": "above the obstacle",
                "reference_kind": "transit_anchor",
                "geometric_meaning": "midway between cup and bowl, above the bottle",
            },
            {
                "step_id": "s2",
                "type": "RELEASE",
                "desc": "bowl center",
                "reference_kind": "destination_support",
                "geometric_meaning": "the interior center of the bowl",
            },
        ],
    }
    base.update(overrides)
    return base


def _tool_stage_a_json(**overrides) -> dict:
    """Fixed 4-step tool-use plan shape (ZeroDex App. D.2 PART B / schema.py
    TOOL_USE_STEP_ORDER): GRASP, FUNCTIONAL_TIP, APPLY_ACTION, RELEASE."""
    base = {
        "schema_version": "0",
        "view_description": "A brush and an apple sit on a board next to a mat.",
        "mode": "tool",
        "target": {"name": "apple", "visibility": "visible"},
        "destination": {"name": "orange mat", "visibility": "visible"},
        "tool": {"name": "brush", "visibility": "visible"},
        "status": "ready",
        "reason_codes": [],
        "steps": [
            {
                "step_id": "s0",
                "type": "GRASP",
                "desc": "brush handle",
                "reference_kind": "grasp_contact",
                "geometric_meaning": "the brush's handle, opposite its bristles",
            },
            {
                "step_id": "s0b",
                "type": "FUNCTIONAL_TIP",
                "desc": "brush bristle tip",
                "reference_kind": "tool_functional_tip",
                "geometric_meaning": "the tip of the brush's bristles",
            },
            {
                "step_id": "s1",
                "type": "APPLY_ACTION",
                "desc": "apple contact point",
                "reference_kind": "transit_anchor",
                "geometric_meaning": "the side of the apple the bristles will sweep",
            },
            {
                "step_id": "s2",
                "type": "RELEASE",
                "desc": "orange mat center",
                "reference_kind": "destination_support",
                "geometric_meaning": "the center of the orange mat",
            },
        ],
    }
    base.update(overrides)
    return base


def _stage_b_json(status: str = "localized") -> list:
    if status == "localized":
        return [
            {"step_id": "s0", "point_yx_norm1000": [412, 388], "point_status": "localized", "reason_codes": []},
            {"step_id": "s1", "point_yx_norm1000": [300, 520], "point_status": "localized", "reason_codes": []},
            {"step_id": "s2", "point_yx_norm1000": [455, 690], "point_status": "localized", "reason_codes": []},
        ]
    return [
        {"step_id": "s0", "point_yx_norm1000": None, "point_status": "unavailable", "reason_codes": ["OCCLUDED"]},
        {"step_id": "s1", "point_yx_norm1000": None, "point_status": "unavailable", "reason_codes": ["OCCLUDED"]},
        {"step_id": "s2", "point_yx_norm1000": None, "point_status": "unavailable", "reason_codes": ["OCCLUDED"]},
    ]


class StageAParsingTest(unittest.TestCase):
    def test_valid_ready_plan_parses(self) -> None:
        plan = StageAPlan.from_json_text(json.dumps(_stage_a_json()))
        self.assertEqual(plan.mode.value, "pick")
        self.assertEqual(plan.target.name, "white cup")
        self.assertEqual(len(plan.steps), 3)
        self.assertFalse(plan.unsupported_by_executor)

    def test_strips_one_markdown_fence(self) -> None:
        fenced = "```json\n" + json.dumps(_stage_a_json()) + "\n```"
        plan = StageAPlan.from_json_text(strip_code_fence(fenced))
        self.assertEqual(plan.status.value, "ready")

    def test_ready_without_target_is_rejected(self) -> None:
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(json.dumps(_stage_a_json(target=None)))

    def test_abstain_without_reason_codes_is_rejected(self) -> None:
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(
                json.dumps(_stage_a_json(status="abstain", reason_codes=[], steps=[]))
            )

    def test_unknown_reason_code_is_rejected(self) -> None:
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(
                json.dumps(_stage_a_json(status="abstain", reason_codes=["MADE_UP_CODE"], steps=[]))
            )

    def test_image_relative_language_is_rejected(self) -> None:
        bad = _stage_a_json()
        bad["steps"][1]["geometric_meaning"] = "left of the image, near the bottle"
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(json.dumps(bad))

    def test_duplicate_step_id_is_rejected(self) -> None:
        bad = _stage_a_json()
        bad["steps"][1]["step_id"] = "s0"
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(json.dumps(bad))

    def test_tool_mode_is_unsupported_by_executor(self) -> None:
        plan = StageAPlan.from_json_text(json.dumps(_tool_stage_a_json()))
        self.assertTrue(plan.unsupported_by_executor)

    def test_tool_mode_requires_four_steps(self) -> None:
        # A 3-step pick-shape plan under mode="tool" is rejected — tool-use
        # must produce GRASP, FUNCTIONAL_TIP, APPLY_ACTION, RELEASE/HOLD.
        d = _stage_a_json(mode="tool", tool={"name": "brush", "visibility": "visible"})
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(json.dumps(d))

    def test_tool_mode_without_tool_role_is_rejected(self) -> None:
        d = _tool_stage_a_json(tool=None)
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(json.dumps(d))

    def test_tool_mode_wrong_fourth_step_type_is_rejected(self) -> None:
        d = _tool_stage_a_json()
        d["steps"][3]["type"] = "WAYPOINT"
        with self.assertRaises(ParseError):
            StageAPlan.from_json_text(json.dumps(d))

    def test_tool_mode_accepts_hold_as_fourth_step(self) -> None:
        d = _tool_stage_a_json()
        d["steps"][3]["type"] = "HOLD"
        plan = StageAPlan.from_json_text(json.dumps(d))
        self.assertEqual(plan.steps[3].type.value, "HOLD")


class StageBParsingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.stage_a = StageAPlan.from_json_text(json.dumps(_stage_a_json()))

    def test_localized_points_parse_and_preserve_order(self) -> None:
        located = parse_stage_b_response(json.dumps(_stage_b_json()), self.stage_a)
        self.assertEqual([s.spec.step_id for s in located], ["s0", "s1", "s2"])
        self.assertEqual(located[0].point_status, PointStatus.LOCALIZED)
        self.assertEqual(located[0].point_yx_norm1000, (412.0, 388.0))

    def test_typed_abstention_when_unavailable(self) -> None:
        located = parse_stage_b_response(json.dumps(_stage_b_json("unavailable")), self.stage_a)
        self.assertTrue(all(s.point_status == PointStatus.UNAVAILABLE for s in located))
        self.assertTrue(all(s.point_yx_norm1000 is None for s in located))

    def test_step_id_reordering_is_rejected(self) -> None:
        entries = _stage_b_json()
        entries[0], entries[1] = entries[1], entries[0]
        with self.assertRaises(ParseError):
            parse_stage_b_response(json.dumps(entries), self.stage_a)

    def test_out_of_range_point_is_rejected(self) -> None:
        entries = _stage_b_json()
        entries[0]["point_yx_norm1000"] = [1200, 388]
        with self.assertRaises(ParseError):
            parse_stage_b_response(json.dumps(entries), self.stage_a)

    def test_bool_as_number_is_rejected(self) -> None:
        entries = _stage_b_json()
        entries[0]["point_yx_norm1000"] = [True, 388]
        with self.assertRaises(ParseError):
            parse_stage_b_response(json.dumps(entries), self.stage_a)

    def test_nan_is_rejected(self) -> None:
        raw = json.dumps(_stage_b_json()).replace("412", "NaN")
        with self.assertRaises(ParseError):
            parse_stage_b_response(raw, self.stage_a)

    def test_localized_without_point_is_rejected(self) -> None:
        entries = _stage_b_json()
        entries[0]["point_yx_norm1000"] = None
        with self.assertRaises(ParseError):
            parse_stage_b_response(json.dumps(entries), self.stage_a)

    def test_unavailable_without_reason_codes_is_rejected(self) -> None:
        entries = _stage_b_json("unavailable")
        entries[0]["reason_codes"] = []
        with self.assertRaises(ParseError):
            parse_stage_b_response(json.dumps(entries), self.stage_a)


class MergePlanTest(unittest.TestCase):
    def test_merge_round_trips_to_json(self) -> None:
        stage_a = StageAPlan.from_json_text(json.dumps(_stage_a_json()))
        located = parse_stage_b_response(json.dumps(_stage_b_json()), stage_a)
        plan = merge_plan(
            scene_id="s2_03",
            instruction="put the cup in the bowl",
            backend="gemini-robotics-er-2-preview",
            stage_a_plan=stage_a,
            located_steps=located,
            meta=Meta(latency_s=1.2, n_calls=2, parse_retries=0),
        )
        as_json = plan.to_json()
        self.assertEqual(as_json["scene_id"], "s2_03")
        self.assertEqual(len(as_json["steps"]), 3)
        self.assertEqual(as_json["steps"][0]["point_yx_norm1000"], [412.0, 388.0])
        json.dumps(as_json)  # must be JSON-serializable


def _single_call_json(point_status: str = "localized") -> dict:
    d = _stage_a_json()
    points = _stage_b_json(point_status)
    for step, point_entry in zip(d["steps"], points):
        step["point_yx_norm1000"] = point_entry["point_yx_norm1000"]
        step["point_status"] = point_entry["point_status"]
        step["reason_codes"] = point_entry["reason_codes"]
    return d


class SingleCallParsingTest(unittest.TestCase):
    def test_valid_single_call_parses_both_halves(self) -> None:
        stage_a, located = parse_single_call_response(json.dumps(_single_call_json()))
        self.assertEqual(stage_a.status.value, "ready")
        self.assertEqual(len(located), 3)
        self.assertEqual(located[0].point_status, PointStatus.LOCALIZED)
        self.assertEqual(located[0].point_yx_norm1000, (412.0, 388.0))

    def test_typed_abstention_in_single_call(self) -> None:
        _, located = parse_single_call_response(json.dumps(_single_call_json("unavailable")))
        self.assertTrue(all(s.point_status == PointStatus.UNAVAILABLE for s in located))

    def test_header_failure_propagates_as_single_call_stage(self) -> None:
        bad = _single_call_json()
        bad["mode"] = "not_a_real_mode"
        with self.assertRaises(ParseError) as ctx:
            parse_single_call_response(json.dumps(bad))
        self.assertEqual(ctx.exception.stage, "single_call")

    def test_out_of_range_point_in_single_call_is_rejected(self) -> None:
        bad = _single_call_json()
        bad["steps"][0]["point_yx_norm1000"] = [-5, 388]
        with self.assertRaises(ParseError):
            parse_single_call_response(json.dumps(bad))


def _stage_0_json(**overrides) -> dict:
    base = {
        "scene": "an apple on a board, a brush, an orange mat on one side and a green mat on the other",
        "holding": {"is_holding": False, "object": None},
        "status": "nothing moved yet",
        "tasks": ["use the brush to sweep the apple onto the orange mat"],
    }
    base.update(overrides)
    return base


class Stage0ParsingTest(unittest.TestCase):
    def test_valid_plan_parses(self) -> None:
        plan = parse_stage_0_response(json.dumps(_stage_0_json()))
        self.assertIsInstance(plan, ScenarioPlan)
        self.assertFalse(plan.is_holding)
        self.assertIsNone(plan.held_object)
        self.assertEqual(plan.tasks, ("use the brush to sweep the apple onto the orange mat",))

    def test_strips_one_markdown_fence(self) -> None:
        fenced = "```json\n" + json.dumps(_stage_0_json()) + "\n```"
        plan = parse_stage_0_response(strip_code_fence(fenced))
        self.assertEqual(len(plan.tasks), 1)

    def test_empty_tasks_is_not_an_error(self) -> None:
        plan = parse_stage_0_response(json.dumps(_stage_0_json(tasks=[], status="done")))
        self.assertEqual(plan.tasks, ())

    def test_holding_true_requires_object(self) -> None:
        bad = _stage_0_json(holding={"is_holding": True, "object": None})
        with self.assertRaises(ParseError):
            parse_stage_0_response(json.dumps(bad))

    def test_holding_false_requires_null_object(self) -> None:
        bad = _stage_0_json(holding={"is_holding": False, "object": "brush"})
        with self.assertRaises(ParseError):
            parse_stage_0_response(json.dumps(bad))

    def test_non_string_task_is_rejected(self) -> None:
        bad = _stage_0_json(tasks=[123])
        with self.assertRaises(ParseError):
            parse_stage_0_response(json.dumps(bad))

    def test_malformed_json_is_rejected(self) -> None:
        with self.assertRaises(ParseError):
            parse_stage_0_response("{not json")


def _localized_affordance_json(**overrides) -> dict:
    d = {
        "status": "localized",
        "affordance_description": "kettle handle",
        "affordance_point_yx_norm1000": [300, 400],
        "affordance_bbox_yx_norm1000": [250, 350, 350, 450],
        "reason_codes": [],
    }
    d.update(overrides)
    return d


class ParseGraspAffordanceResponseTest(unittest.TestCase):
    def test_localized_round_trips_all_fields(self) -> None:
        region = parse_grasp_affordance_response(json.dumps(_localized_affordance_json()))
        self.assertIsInstance(region, AffordanceRegion)
        self.assertEqual(region.status, PointStatus.LOCALIZED)
        self.assertEqual(region.description, "kettle handle")
        self.assertEqual(region.point_yx_norm1000, (300.0, 400.0))
        self.assertEqual(region.bbox_yx_norm1000, (250.0, 350.0, 350.0, 450.0))
        self.assertEqual(region.reason_codes, [])

    def test_unavailable_requires_reason_codes(self) -> None:
        raw = json.dumps({
            "status": "unavailable", "affordance_description": None,
            "affordance_point_yx_norm1000": None, "affordance_bbox_yx_norm1000": None,
            "reason_codes": [],
        })
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(raw)

    def test_unavailable_with_reason_code_parses(self) -> None:
        raw = json.dumps({
            "status": "unavailable", "affordance_description": None,
            "affordance_point_yx_norm1000": None, "affordance_bbox_yx_norm1000": None,
            "reason_codes": ["OCCLUDED"],
        })
        region = parse_grasp_affordance_response(raw)
        self.assertEqual(region.status, PointStatus.UNAVAILABLE)
        self.assertIsNone(region.bbox_yx_norm1000)
        self.assertEqual(region.reason_codes, ["OCCLUDED"])

    def test_unavailable_with_a_bbox_is_rejected(self) -> None:
        # Rule 5 in prompt form: an UNAVAILABLE response must not smuggle a
        # guessed box in alongside the abstention.
        raw = json.dumps(_localized_affordance_json(status="unavailable", reason_codes=["OCCLUDED"]))
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(raw)

    def test_localized_requires_bbox(self) -> None:
        d = _localized_affordance_json()
        d["affordance_bbox_yx_norm1000"] = None
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(json.dumps(d))

    def test_bbox_must_be_top_left_then_bottom_right(self) -> None:
        d = _localized_affordance_json(affordance_bbox_yx_norm1000=[350, 450, 250, 350])  # reversed
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(json.dumps(d))

    def test_bbox_out_of_range_rejected(self) -> None:
        d = _localized_affordance_json(affordance_bbox_yx_norm1000=[250, 350, 350, 1500])
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(json.dumps(d))

    def test_bbox_wrong_length_rejected(self) -> None:
        d = _localized_affordance_json(affordance_bbox_yx_norm1000=[250, 350, 350])
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(json.dumps(d))

    def test_unknown_reason_code_rejected(self) -> None:
        raw = json.dumps({
            "status": "unavailable", "affordance_description": None,
            "affordance_point_yx_norm1000": None, "affordance_bbox_yx_norm1000": None,
            "reason_codes": ["NOT_A_REAL_CODE"],
        })
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(raw)

    def test_invalid_status_rejected(self) -> None:
        d = _localized_affordance_json(status="maybe")
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response(json.dumps(d))

    def test_malformed_json_rejected(self) -> None:
        with self.assertRaises(ParseError):
            parse_grasp_affordance_response("{not json")


class CoordinateAdapterTest(unittest.TestCase):
    def test_new_adapter_maps_max_to_last_pixel_center(self) -> None:
        x_px, y_px = pixel_from_norm1000((1000, 1000), width=1280, height=720, adapter="new")
        self.assertAlmostEqual(x_px, 1279.0)
        self.assertAlmostEqual(y_px, 719.0)

    def test_legacy_adapter_can_reach_width_itself(self) -> None:
        x_px, y_px = pixel_from_norm1000((1000, 1000), width=1280, height=720, adapter="legacy")
        self.assertAlmostEqual(x_px, 1280.0)
        self.assertAlmostEqual(y_px, 720.0)

    def test_zero_maps_to_zero_in_both_adapters(self) -> None:
        for adapter in ("new", "legacy"):
            x_px, y_px = pixel_from_norm1000((0, 0), width=1280, height=720, adapter=adapter)
            self.assertEqual((x_px, y_px), (0.0, 0.0))

    def test_unknown_adapter_rejected(self) -> None:
        with self.assertRaises(ValueError):
            pixel_from_norm1000((0, 0), width=10, height=10, adapter="bogus")


if __name__ == "__main__":
    unittest.main()
