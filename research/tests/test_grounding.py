from __future__ import annotations

import json
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from PIL import Image  # noqa: E402

from mpg.grounding import (  # noqa: E402
    GroundingFailure,
    run_grasp_affordance,
    run_single_call,
    run_stage_0,
    run_stage_a,
    run_stage_b,
)
from mpg.vlm.base import QueryResult  # noqa: E402


def _stage_a_json() -> dict:
    return {
        "schema_version": "0",
        "view_description": "A white cup sits on a table next to a blue bowl.",
        "mode": "pick",
        "target": {"name": "white cup", "visibility": "visible"},
        "destination": {"name": "blue bowl", "visibility": "visible"},
        "tool": None,
        "status": "ready",
        "reason_codes": [],
        "steps": [
            {"step_id": "s0", "type": "GRASP", "desc": "cup body", "reference_kind": "grasp_contact",
             "geometric_meaning": "the cup's handle-side wall"},
            {"step_id": "s1", "type": "WAYPOINT", "desc": "above the obstacle", "reference_kind": "transit_anchor",
             "geometric_meaning": "midway between cup and bowl"},
            {"step_id": "s2", "type": "RELEASE", "desc": "bowl center", "reference_kind": "destination_support",
             "geometric_meaning": "the interior center of the bowl"},
        ],
    }


def _stage_b_json() -> list:
    return [
        {"step_id": "s0", "point_yx_norm1000": [412, 388], "point_status": "localized", "reason_codes": []},
        {"step_id": "s1", "point_yx_norm1000": [300, 520], "point_status": "localized", "reason_codes": []},
        {"step_id": "s2", "point_yx_norm1000": [455, 690], "point_status": "localized", "reason_codes": []},
    ]


class _FakeBackend:
    """Returns canned raw_text values in sequence, one per .query() call."""

    def __init__(self, responses: list[str]):
        self._responses = list(responses)
        self.calls: list[str] = []  # prompts received, for assertions

    def query(self, image, prompt, *, replicate_id: str = ""):
        self.calls.append(prompt)
        raw_text = self._responses.pop(0)
        return QueryResult(raw_text=raw_text, latency_s=0.01, cache_hit=False, model="fake")


def _image() -> Image.Image:
    return Image.new("RGB", (4, 4))


def _stage_0_json() -> dict:
    return {
        "scene": "an apple on a board, a brush, an orange mat and a green mat",
        "holding": {"is_holding": False, "object": None},
        "status": "nothing moved yet",
        "tasks": ["use the brush to sweep the apple onto the orange mat"],
    }


class RunStage0Test(unittest.TestCase):
    def test_succeeds_on_first_try(self) -> None:
        backend = _FakeBackend([json.dumps(_stage_0_json())])
        plan, info = run_stage_0(backend, _image(), "move the object to the matched color")
        self.assertEqual(plan.tasks, ("use the brush to sweep the apple onto the orange mat",))
        self.assertEqual(info.n_calls, 1)
        self.assertEqual(info.parse_retries, 0)

    def test_retries_once_on_bad_json_then_succeeds(self) -> None:
        backend = _FakeBackend(["not json at all", json.dumps(_stage_0_json())])
        plan, info = run_stage_0(backend, _image(), "move the object to the matched color")
        self.assertEqual(info.n_calls, 2)
        self.assertEqual(info.parse_retries, 1)
        self.assertIn("FAILED TO PARSE", backend.calls[1])

    def test_raises_grounding_failure_after_exhausting_retries(self) -> None:
        backend = _FakeBackend(["not json", "still not json"])
        with self.assertRaises(GroundingFailure) as ctx:
            run_stage_0(backend, _image(), "move the object to the matched color", max_retries=1)
        self.assertEqual(ctx.exception.stage, "stage_0")
        self.assertEqual(ctx.exception.n_calls, 2)

    def test_goal_is_substituted_into_prompt(self) -> None:
        backend = _FakeBackend([json.dumps(_stage_0_json())])
        run_stage_0(backend, _image(), "move the object to the matched color")
        self.assertIn("move the object to the matched color", backend.calls[0])
        self.assertNotIn("{goal}", backend.calls[0])


class RunStageATest(unittest.TestCase):
    def test_succeeds_on_first_try(self) -> None:
        backend = _FakeBackend([json.dumps(_stage_a_json())])
        plan, info = run_stage_a(backend, _image(), "put the cup in the bowl")
        self.assertEqual(plan.status.value, "ready")
        self.assertEqual(info.n_calls, 1)
        self.assertEqual(info.parse_retries, 0)

    def test_retries_once_on_bad_json_then_succeeds(self) -> None:
        backend = _FakeBackend(["not json at all", json.dumps(_stage_a_json())])
        plan, info = run_stage_a(backend, _image(), "put the cup in the bowl")
        self.assertEqual(plan.status.value, "ready")
        self.assertEqual(info.n_calls, 2)
        self.assertEqual(info.parse_retries, 1)
        # the retry prompt must include the original instruction context
        self.assertIn("FAILED TO PARSE", backend.calls[1])

    def test_raises_grounding_failure_after_exhausting_retries(self) -> None:
        backend = _FakeBackend(["not json", "still not json"])
        with self.assertRaises(GroundingFailure) as ctx:
            run_stage_a(backend, _image(), "put the cup in the bowl", max_retries=1)
        self.assertEqual(ctx.exception.stage, "stage_a")
        self.assertEqual(ctx.exception.n_calls, 2)
        self.assertEqual(len(ctx.exception.raw_responses), 2)

    def test_instruction_is_substituted_into_prompt(self) -> None:
        backend = _FakeBackend([json.dumps(_stage_a_json())])
        run_stage_a(backend, _image(), "put the red cup in the blue bowl")
        self.assertIn("put the red cup in the blue bowl", backend.calls[0])
        self.assertNotIn("{instruction}", backend.calls[0])


def _grasp_affordance_json() -> dict:
    return {
        "status": "localized",
        "affordance_description": "kettle handle",
        "affordance_point_yx_norm1000": [300, 400],
        "affordance_bbox_yx_norm1000": [250, 350, 350, 450],
        "reason_codes": [],
    }


class RunGraspAffordanceTest(unittest.TestCase):
    def test_succeeds_on_first_try(self) -> None:
        backend = _FakeBackend([json.dumps(_grasp_affordance_json())])
        region, info = run_grasp_affordance(
            backend, _image(), grasp_object="kettle", grasp_hint="pour water from the kettle"
        )
        self.assertEqual(region.description, "kettle handle")
        self.assertEqual(info.n_calls, 1)
        self.assertEqual(info.parse_retries, 0)

    def test_retries_once_on_bad_json_then_succeeds(self) -> None:
        backend = _FakeBackend(["not json at all", json.dumps(_grasp_affordance_json())])
        region, info = run_grasp_affordance(
            backend, _image(), grasp_object="kettle", grasp_hint="pour water from the kettle"
        )
        self.assertEqual(info.n_calls, 2)
        self.assertEqual(info.parse_retries, 1)
        self.assertIn("FAILED TO PARSE", backend.calls[1])

    def test_raises_grounding_failure_after_exhausting_retries(self) -> None:
        backend = _FakeBackend(["not json", "still not json"])
        with self.assertRaises(GroundingFailure) as ctx:
            run_grasp_affordance(
                backend, _image(), grasp_object="kettle", grasp_hint="pour water",
                max_retries=1,
            )
        self.assertEqual(ctx.exception.stage, "grasp_affordance")
        self.assertEqual(ctx.exception.n_calls, 2)

    def test_grasp_object_and_hint_are_substituted_into_prompt(self) -> None:
        backend = _FakeBackend([json.dumps(_grasp_affordance_json())])
        run_grasp_affordance(
            backend, _image(), grasp_object="kettle", grasp_hint="pour water from the kettle"
        )
        self.assertIn("kettle", backend.calls[0])
        self.assertIn("pour water from the kettle", backend.calls[0])
        self.assertNotIn("{grasp_object}", backend.calls[0])
        self.assertNotIn("{grasp_hint}", backend.calls[0])

    def test_correct_abstention_is_not_retried(self) -> None:
        # A well-formed "unavailable" response is a valid parse, not a
        # format failure -- it must not consume a retry or be resent.
        raw = json.dumps({
            "status": "unavailable", "affordance_description": None,
            "affordance_point_yx_norm1000": None, "affordance_bbox_yx_norm1000": None,
            "reason_codes": ["OCCLUDED"],
        })
        backend = _FakeBackend([raw])
        region, info = run_grasp_affordance(
            backend, _image(), grasp_object="kettle", grasp_hint="pour water"
        )
        self.assertEqual(region.reason_codes, ["OCCLUDED"])
        self.assertEqual(info.n_calls, 1)
        self.assertEqual(info.parse_retries, 0)


class RunStageBTest(unittest.TestCase):
    def setUp(self) -> None:
        from mpg.schema import StageAPlan

        self.stage_a_plan = StageAPlan.from_json_text(json.dumps(_stage_a_json()))

    def test_succeeds_and_substitutes_plan_json(self) -> None:
        backend = _FakeBackend([json.dumps(_stage_b_json())])
        located, info = run_stage_b(backend, _image(), self.stage_a_plan)
        self.assertEqual(len(located), 3)
        self.assertEqual(info.n_calls, 1)
        self.assertNotIn("{plan_steps_json}", backend.calls[0])
        self.assertIn("white cup", backend.calls[0])  # target name substituted

    def test_retries_on_step_id_mismatch(self) -> None:
        bad = _stage_b_json()
        bad[0]["step_id"] = "wrong"
        backend = _FakeBackend([json.dumps(bad), json.dumps(_stage_b_json())])
        located, info = run_stage_b(backend, _image(), self.stage_a_plan)
        self.assertEqual(info.parse_retries, 1)
        self.assertEqual(located[0].spec.step_id, "s0")


class DeriveGraspHintTest(unittest.TestCase):
    """derive_grasp_hint() bridges run_stage_b's located points to
    run_grasp_affordance()'s (grasp_object, grasp_hint) arguments -- see the
    function's docstring and docs/progress/grasp_motion_progress_report.md's
    2026-09-20 revision note for why this glue did not exist before."""

    def setUp(self) -> None:
        from mpg.schema import StageAPlan

        self.stage_a_plan = StageAPlan.from_json_text(json.dumps(_stage_a_json()))

    def test_derives_grasp_object_and_hint_from_the_grasp_step(self) -> None:
        from mpg.grounding import derive_grasp_hint

        backend = _FakeBackend([json.dumps(_stage_b_json())])
        located, _ = run_stage_b(backend, _image(), self.stage_a_plan)
        grasp_object, grasp_hint = derive_grasp_hint(located, instruction="put the cup in the bowl")
        # _stage_a_json()'s s0 step: desc="cup body", geometric_meaning="the cup's
        # handle-side wall" -- both must show up, not be reinvented by the caller.
        self.assertEqual(grasp_object, "cup body")
        self.assertIn("put the cup in the bowl", grasp_hint)
        self.assertIn("the cup's handle-side wall", grasp_hint)

    def test_raises_when_no_grasp_step_present(self) -> None:
        from mpg.grounding import NoGraspStepError, derive_grasp_hint

        backend = _FakeBackend([json.dumps(_stage_b_json())])
        located, _ = run_stage_b(backend, _image(), self.stage_a_plan)
        non_grasp_only = tuple(s for s in located if s.spec.type.value != "GRASP")
        self.assertLess(len(non_grasp_only), len(located))  # sanity: we actually dropped one
        with self.assertRaises(NoGraspStepError):
            derive_grasp_hint(non_grasp_only, instruction="put the cup in the bowl")

    def test_raises_when_grasp_step_is_unavailable(self) -> None:
        from mpg.grounding import NoGraspStepError, derive_grasp_hint

        unavailable_stage_b = _stage_b_json()
        unavailable_stage_b[0] = {
            "step_id": "s0", "point_yx_norm1000": None,
            "point_status": "unavailable", "reason_codes": ["TARGET_NOT_VISIBLE"],
        }
        backend = _FakeBackend([json.dumps(unavailable_stage_b)])
        located, _ = run_stage_b(backend, _image(), self.stage_a_plan)
        with self.assertRaises(NoGraspStepError):
            derive_grasp_hint(located, instruction="put the cup in the bowl")


class RunSingleCallTest(unittest.TestCase):
    def test_succeeds_on_first_try(self) -> None:
        merged = _stage_a_json()
        for step, point_entry in zip(merged["steps"], _stage_b_json()):
            step.update(
                point_yx_norm1000=point_entry["point_yx_norm1000"],
                point_status=point_entry["point_status"],
                reason_codes=point_entry["reason_codes"],
            )
        backend = _FakeBackend([json.dumps(merged)])
        stage_a_plan, located, info = run_single_call(backend, _image(), "put the cup in the bowl")
        self.assertEqual(stage_a_plan.status.value, "ready")
        self.assertEqual(len(located), 3)
        self.assertEqual(info.n_calls, 1)


if __name__ == "__main__":
    unittest.main()
