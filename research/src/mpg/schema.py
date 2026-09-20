"""Plan JSON v0: schema, strict parsing, and the coordinate adapter.

Extends the sketch in AGENTS.md Sec 2 (Phase 2) with the role-visibility and
reason-code fields designed in docs/prompt_strategy.md — written after the
AGENTS.md sketch, so this is the more detailed, current source of truth.
target/destination/tool remain readable as plain strings via Role.name for
compatibility with that sketch.

No VLM calls here; this module only defines the contract and validates text
already returned by a backend. See docs/prompt_strategy.md Sec 4 for the
parser pipeline this module implements: raw -> strip one fence -> strict
JSON -> schema -> semantic consistency -> coordinate validation.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from enum import Enum
from typing import Any

DEFAULT_COORD_NORM_MAX = 1000


class ParseError(ValueError):
    """A response failed at a named stage of the parser pipeline.

    AGENTS.md rule 15 (no false-green): every failure must say which stage
    it failed at and keep the raw text, so a run's failures.jsonl can show
    *why* without re-deriving it from the exception message alone.
    """

    def __init__(self, stage: str, reason: str, raw: str):
        super().__init__(f"[{stage}] {reason}")
        self.stage = stage
        self.reason = reason
        self.raw = raw


class Visibility(str, Enum):
    VISIBLE = "visible"
    PARTIAL = "partial"
    NOT_VISIBLE = "not_visible"
    UNCERTAIN = "uncertain"


class Mode(str, Enum):
    PICK = "pick"
    TOOL = "tool"
    UNSUPPORTED = "unsupported"


class PlanStatus(str, Enum):
    READY = "ready"
    ABSTAIN = "abstain"
    UNSUPPORTED = "unsupported"


class StepType(str, Enum):
    GRASP = "GRASP"
    WAYPOINT = "WAYPOINT"
    RELEASE = "RELEASE"
    # Tool-use primitives (AGENTS.md Sec 2.3, ZeroDex App. D.2 PART B):
    # grasp -> functional tip -> apply_action -> release/hold. Parsed and
    # stored, flagged unsupported_by_executor; the executor only handles
    # pick-and-place.
    FUNCTIONAL_TIP = "FUNCTIONAL_TIP"
    APPLY_ACTION = "APPLY_ACTION"
    HOLD = "HOLD"


# Pick-and-place is exactly this order; tool-use plans are stored but never
# match this, so they fall out of the executor-supported check for free.
PICK_PLACE_STEP_ORDER = (StepType.GRASP, StepType.WAYPOINT, StepType.RELEASE)

# Tool-use is GRASP, FUNCTIONAL_TIP, APPLY_ACTION, then a 4th step that is
# either RELEASE or HOLD (ZeroDex App. D.2 PART B step 2: "RELEASE or HOLD
# --- pick whichever fits"), so the 4th type is checked separately rather
# than folded into one fixed tuple.
TOOL_USE_STEP_ORDER = (StepType.GRASP, StepType.FUNCTIONAL_TIP, StepType.APPLY_ACTION)
TOOL_USE_FINAL_STEP_TYPES = (StepType.RELEASE, StepType.HOLD)


class ReferenceKind(str, Enum):
    GRASP_CONTACT = "grasp_contact"
    OBJECT_BOTTOM_CENTER = "object_bottom_center"
    DESTINATION_SUPPORT = "destination_support"
    TRANSIT_ANCHOR = "transit_anchor"
    TOOL_FUNCTIONAL_TIP = "tool_functional_tip"


class PointStatus(str, Enum):
    LOCALIZED = "localized"
    # Rule 5: never guess a coordinate for an absent/occluded reference.
    # This is the typed alternative to null-with-no-explanation.
    UNAVAILABLE = "unavailable"


# Controlled vocabulary so a parser catches a model inventing its own reason
# string, rather than silently accepting arbitrary text as if it were a
# validated code (rule 15: don't paper over a bad response as success).
REASON_CODES = frozenset(
    {
        "TARGET_NOT_VISIBLE",
        "DESTINATION_NOT_VISIBLE",
        "TOOL_REQUIRED_UNSUPPORTED",
        "AMBIGUOUS_REFERENT",
        "INSUFFICIENT_EVIDENCE",
        "OCCLUDED",
        "OUT_OF_FRAME",
        "UNCERTAIN_LOCALIZATION",
        "FORMAT_RETRY_EXHAUSTED",
        "MODEL_ABSTAINED",
    }
)


def _check_reason_codes(stage: str, raw: str, codes: Any) -> list[str]:
    if not isinstance(codes, list) or not all(isinstance(c, str) for c in codes):
        raise ParseError(stage, "reason_codes must be a list of strings", raw)
    unknown = sorted(set(codes) - REASON_CODES)
    if unknown:
        raise ParseError(stage, f"unknown reason codes: {unknown}", raw)
    return codes


def _require_str(stage: str, raw: str, d: dict, key: str) -> str:
    value = d.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ParseError(stage, f"'{key}' must be a non-empty string", raw)
    return value


@dataclass(frozen=True)
class ScenarioPlan:
    """Stage 0 output (ZeroDex App. D.2 "Long-horizon Planner", adapted for
    a single grounding call — see prompts/stage_0_planner.txt provenance
    header for what was dropped/kept). `tasks` is an ordered list of
    candidate atomic tasks; this project consumes only tasks[0] per image
    since it runs grounding once, not as a closed-loop executor. An empty
    tasks list means the goal already appears satisfied — not an error."""

    scene: str
    is_holding: bool
    held_object: str | None
    status: str
    tasks: tuple[str, ...]

    def to_json(self) -> dict:
        return {
            "scene": self.scene,
            "holding": {"is_holding": self.is_holding, "object": self.held_object},
            "status": self.status,
            "tasks": list(self.tasks),
        }


def parse_stage_0_response(raw_text: str) -> ScenarioPlan:
    stage = "stage_0"
    raw = raw_text
    text = strip_code_fence(raw_text)
    d = parse_json_strict(stage, text)
    if not isinstance(d, dict):
        raise ParseError(stage, "top-level response must be a JSON object", raw)

    scene = _require_str(stage, raw, d, "scene")
    status = _require_str(stage, raw, d, "status")

    holding = d.get("holding")
    if not isinstance(holding, dict):
        raise ParseError(stage, "'holding' must be an object", raw)
    is_holding = holding.get("is_holding")
    if not isinstance(is_holding, bool):
        raise ParseError(stage, "'holding.is_holding' must be a bool", raw)
    held_object = holding.get("object")
    if held_object is not None and not isinstance(held_object, str):
        raise ParseError(stage, "'holding.object' must be a string or null", raw)
    if is_holding and held_object is None:
        raise ParseError(stage, "'holding.is_holding'=true requires a non-null object", raw)
    if not is_holding and held_object is not None:
        raise ParseError(stage, "'holding.is_holding'=false requires object=null", raw)

    tasks_raw = d.get("tasks")
    if not isinstance(tasks_raw, list) or not all(
        isinstance(t, str) and t.strip() for t in tasks_raw
    ):
        raise ParseError(stage, "'tasks' must be a list of non-empty strings", raw)

    return ScenarioPlan(
        scene=scene,
        is_holding=is_holding,
        held_object=held_object,
        status=status,
        tasks=tuple(tasks_raw),
    )


@dataclass(frozen=True)
class Role:
    """target / destination / tool. A tool is never the robot arm/gripper
    (AGENTS.md Sec 2.3, App D.2 prompt rule)."""

    name: str
    visibility: Visibility

    def to_json(self) -> dict:
        return {"name": self.name, "visibility": self.visibility.value}

    @staticmethod
    def from_dict(stage: str, raw: str, d: Any, *, field_name: str) -> "Role | None":
        if d is None:
            return None
        if not isinstance(d, dict):
            raise ParseError(stage, f"'{field_name}' must be an object or null", raw)
        name = _require_str(stage, raw, d, "name")
        visibility_raw = d.get("visibility")
        try:
            visibility = Visibility(visibility_raw)
        except ValueError:
            raise ParseError(
                stage, f"'{field_name}.visibility' is not a valid Visibility", raw
            ) from None
        return Role(name=name, visibility=visibility)


@dataclass(frozen=True)
class PlanStepSpec:
    """A Stage A step, before Stage B has localized it. Stage B must not
    change step_id, type, desc, or geometric_meaning (prompt_strategy.md)."""

    step_id: str
    type: StepType
    desc: str
    reference_kind: ReferenceKind
    geometric_meaning: str

    def to_json(self) -> dict:
        return {
            "step_id": self.step_id,
            "type": self.type.value,
            "desc": self.desc,
            "reference_kind": self.reference_kind.value,
            "geometric_meaning": self.geometric_meaning,
        }

    @staticmethod
    def from_dict(stage: str, raw: str, d: Any) -> "PlanStepSpec":
        if not isinstance(d, dict):
            raise ParseError(stage, "each step must be an object", raw)
        step_id = _require_str(stage, raw, d, "step_id")
        type_raw = d.get("type")
        try:
            step_type = StepType(type_raw)
        except ValueError:
            raise ParseError(stage, f"step {step_id!r}: invalid type {type_raw!r}", raw) from None
        desc = _require_str(stage, raw, d, "desc")
        ref_raw = d.get("reference_kind")
        try:
            reference_kind = ReferenceKind(ref_raw)
        except ValueError:
            raise ParseError(
                stage, f"step {step_id!r}: invalid reference_kind {ref_raw!r}", raw
            ) from None
        geometric_meaning = _require_str(stage, raw, d, "geometric_meaning")
        # App D.2 rule, carried into Stage A: forbid image-relative language.
        lowered = geometric_meaning.lower()
        for banned in ("image-left", "image-right", "image-top", "image-bottom",
                       "left of the image", "right of the image"):
            if banned in lowered:
                raise ParseError(
                    stage,
                    f"step {step_id!r}: geometric_meaning uses forbidden "
                    f"image-relative language ({banned!r})",
                    raw,
                )
        return PlanStepSpec(
            step_id=step_id,
            type=step_type,
            desc=desc,
            reference_kind=reference_kind,
            geometric_meaning=geometric_meaning,
        )


@dataclass(frozen=True)
class StageAPlan:
    schema_version: str
    scene_summary: str
    mode: Mode
    target: Role | None
    destination: Role | None
    tool: Role | None
    status: PlanStatus
    reason_codes: list[str]
    steps: tuple[PlanStepSpec, ...]

    @property
    def unsupported_by_executor(self) -> bool:
        """True for any plan the Phase-2 executor cannot run: tool-use mode,
        or a step sequence other than exactly GRASP, WAYPOINT, RELEASE."""
        if self.mode is not Mode.PICK:
            return True
        return tuple(s.type for s in self.steps) != PICK_PLACE_STEP_ORDER

    def to_json(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "steps": [step.to_json() for step in self.steps],
            "view_description": self.scene_summary,
            "mode": self.mode.value,
            "target": self.target.to_json() if self.target else None,
            "destination": self.destination.to_json() if self.destination else None,
            "tool": self.tool.to_json() if self.tool else None,
            "status": self.status.value,
            "reason_codes": list(self.reason_codes),
        }

    @staticmethod
    def from_json_text(raw_text: str, *, expected_schema_version: str = "0") -> "StageAPlan":
        stage = "stage_a"
        raw = raw_text
        text = strip_code_fence(raw_text)
        d = parse_json_strict(stage, text)
        if not isinstance(d, dict):
            raise ParseError(stage, "top-level response must be a JSON object", raw)

        schema_version = str(d.get("schema_version", expected_schema_version))
        if schema_version != expected_schema_version:
            raise ParseError(
                stage,
                f"schema_version {schema_version!r} != expected {expected_schema_version!r}",
                raw,
            )
        scene_summary = _require_str(stage, raw, d, "view_description")

        mode_raw = d.get("mode")
        try:
            mode = Mode(mode_raw)
        except ValueError:
            raise ParseError(stage, f"invalid mode {mode_raw!r}", raw) from None

        target = Role.from_dict(stage, raw, d.get("target"), field_name="target")
        destination = Role.from_dict(stage, raw, d.get("destination"), field_name="destination")
        tool = Role.from_dict(stage, raw, d.get("tool"), field_name="tool")

        status_raw = d.get("status")
        try:
            status = PlanStatus(status_raw)
        except ValueError:
            raise ParseError(stage, f"invalid status {status_raw!r}", raw) from None

        reason_codes = _check_reason_codes(stage, raw, d.get("reason_codes", []))

        steps_raw = d.get("steps", [])
        if not isinstance(steps_raw, list):
            raise ParseError(stage, "'steps' must be a list", raw)
        steps = tuple(PlanStepSpec.from_dict(stage, raw, s) for s in steps_raw)
        step_ids = [s.step_id for s in steps]
        if len(step_ids) != len(set(step_ids)):
            raise ParseError(stage, f"duplicate step_id in {step_ids}", raw)

        # Semantic consistency (prompt_strategy.md Sec 6): format legality
        # is not the same thing as a legal plan.
        if status is PlanStatus.READY:
            if target is None or target.visibility not in (Visibility.VISIBLE, Visibility.PARTIAL):
                raise ParseError(
                    "semantic", "status=ready requires a visible-or-partial target", raw
                )
            if not steps:
                raise ParseError(stage, "status=ready requires at least one step", raw)
            if mode is Mode.PICK:
                if destination is None or destination.visibility not in (Visibility.VISIBLE, Visibility.PARTIAL):
                    raise ParseError("semantic", "ready pick-and-place requires a visible-or-partial destination", raw)
                if tuple(step.type for step in steps) != PICK_PLACE_STEP_ORDER:
                    raise ParseError("semantic", "ready pick-and-place requires GRASP, WAYPOINT, RELEASE", raw)
            if mode is Mode.TOOL:
                if tool is None or tool.visibility not in (Visibility.VISIBLE, Visibility.PARTIAL):
                    raise ParseError("semantic", "ready tool-use requires a visible-or-partial tool", raw)
                step_types = tuple(step.type for step in steps)
                if len(step_types) != 4 or step_types[:3] != TOOL_USE_STEP_ORDER:
                    raise ParseError(
                        "semantic",
                        "ready tool-use requires GRASP, FUNCTIONAL_TIP, APPLY_ACTION, then RELEASE/HOLD",
                        raw,
                    )
                if step_types[3] not in TOOL_USE_FINAL_STEP_TYPES:
                    raise ParseError(
                        "semantic", "ready tool-use's 4th step must be RELEASE or HOLD", raw
                    )
        if status in (PlanStatus.ABSTAIN, PlanStatus.UNSUPPORTED) and not reason_codes:
            raise ParseError(
                stage, f"status={status.value} requires non-empty reason_codes", raw
            )

        return StageAPlan(
            schema_version=schema_version,
            scene_summary=scene_summary,
            mode=mode,
            target=target,
            destination=destination,
            tool=tool,
            status=status,
            reason_codes=reason_codes,
            steps=steps,
        )


@dataclass(frozen=True)
class LocatedStep:
    """A Stage A step plus Stage B's localization. point_yx_norm1000 is
    (y, x) each in [0, coord_norm_max], or None iff point_status is
    UNAVAILABLE (rule 5: typed abstention, never a guessed coordinate)."""

    spec: PlanStepSpec
    point_yx_norm1000: tuple[float, float] | None
    point_status: PointStatus
    reason_codes: list[str]

    def to_json(self) -> dict:
        out = self.spec.to_json()
        out["point_yx_norm1000"] = (
            list(self.point_yx_norm1000) if self.point_yx_norm1000 is not None else None
        )
        out["point_status"] = self.point_status.value
        out["reason_codes"] = list(self.reason_codes)
        return out


def _validate_point(stage: str, raw: str, step_id: str, value: Any, coord_norm_max: int) -> tuple[float, float]:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(isinstance(v, bool) for v in value)
        or not all(isinstance(v, (int, float)) for v in value)
    ):
        raise ParseError(stage, f"step {step_id!r}: point_yx_norm1000 must be [y, x] numbers", raw)
    y, x = float(value[0]), float(value[1])
    if not (math.isfinite(y) and math.isfinite(x)):
        raise ParseError(stage, f"step {step_id!r}: point_yx_norm1000 has NaN/Inf", raw)
    if not (0 <= y <= coord_norm_max and 0 <= x <= coord_norm_max):
        raise ParseError(
            stage,
            f"step {step_id!r}: point_yx_norm1000={value} out of [0, {coord_norm_max}]",
            raw,
        )
    return (y, x)


def _parse_located_fields(
    stage: str, raw: str, step_id: str, entry: dict, coord_norm_max: int
) -> tuple[tuple[float, float] | None, PointStatus, list[str]]:
    """Shared by parse_stage_b_response and parse_single_call_response:
    validate point_yx_norm1000 / point_status / reason_codes on one entry.
    """
    status_raw = entry.get("point_status")
    try:
        point_status = PointStatus(status_raw)
    except ValueError:
        raise ParseError(
            stage, f"step {step_id!r}: invalid point_status {status_raw!r}", raw
        ) from None

    reason_codes = _check_reason_codes(stage, raw, entry.get("reason_codes", []))
    point_raw = entry.get("point_yx_norm1000")

    if point_status is PointStatus.LOCALIZED:
        if point_raw is None:
            raise ParseError(stage, f"step {step_id!r}: LOCALIZED requires a point", raw)
        point = _validate_point(stage, raw, step_id, point_raw, coord_norm_max)
    else:
        if point_raw is not None:
            raise ParseError(
                stage, f"step {step_id!r}: UNAVAILABLE must have point_yx_norm1000=null", raw
            )
        if not reason_codes:
            raise ParseError(stage, f"step {step_id!r}: UNAVAILABLE requires reason_codes", raw)
        point = None

    return point, point_status, reason_codes


def parse_stage_b_response(
    raw_text: str,
    stage_a_plan: StageAPlan,
    *,
    coord_norm_max: int = DEFAULT_COORD_NORM_MAX,
) -> tuple[LocatedStep, ...]:
    """Parse Stage B output against the *fixed* plan from Stage A.

    Enforces prompt_strategy.md Sec 3: same step IDs, same order, same
    types as Stage A produced — Stage B may only add localization fields.
    """
    stage = "stage_b"
    raw = raw_text
    text = strip_code_fence(raw_text)
    parsed = parse_json_strict(stage, text)
    if not isinstance(parsed, list):
        raise ParseError(stage, "top-level response must be a JSON list", raw)
    if len(parsed) != len(stage_a_plan.steps):
        raise ParseError(
            stage,
            f"got {len(parsed)} located steps, expected {len(stage_a_plan.steps)}",
            raw,
        )

    located: list[LocatedStep] = []
    for spec, entry in zip(stage_a_plan.steps, parsed):
        if not isinstance(entry, dict):
            raise ParseError(stage, f"step {spec.step_id!r}: entry must be an object", raw)
        step_id = entry.get("step_id")
        if step_id != spec.step_id:
            raise ParseError(
                stage, f"step_id mismatch: expected {spec.step_id!r}, got {step_id!r}", raw
            )
        type_raw = entry.get("type")
        if type_raw is not None and type_raw != spec.type.value:
            raise ParseError(
                stage, f"step {spec.step_id!r}: type must not change from Stage A", raw
            )

        point, point_status, reason_codes = _parse_located_fields(
            stage, raw, spec.step_id, entry, coord_norm_max
        )
        located.append(
            LocatedStep(
                spec=spec,
                point_yx_norm1000=point,
                point_status=point_status,
                reason_codes=reason_codes,
            )
        )
    return tuple(located)


def parse_single_call_response(
    raw_text: str,
    *,
    expected_schema_version: str = "0",
    coord_norm_max: int = DEFAULT_COORD_NORM_MAX,
) -> tuple[StageAPlan, tuple[LocatedStep, ...]]:
    """Parse a single-call response (prompts/single_call.txt): task
    grounding and point localization merged into one JSON object.

    Each step object carries BOTH the PlanStepSpec fields and the point
    fields at once, rather than being split across two responses. Reuses
    StageAPlan.from_json_text for the header/steps-without-points pass
    (which already enforces schema_version, role, mode, status, and
    abstention consistency), then re-walks the same raw JSON to pull out
    the point fields the plain StageAPlan parser ignores.
    """
    stage = "single_call"
    raw = raw_text
    text = strip_code_fence(raw_text)
    d = parse_json_strict(stage, text)
    if not isinstance(d, dict):
        raise ParseError(stage, "top-level response must be a JSON object", raw)

    # Defer header/steps-without-points validation to the same logic Stage A
    # uses, re-tagging any failure as this stage for failures.jsonl grouping.
    try:
        stage_a_plan = StageAPlan.from_json_text(text, expected_schema_version=expected_schema_version)
    except ParseError as exc:
        raise ParseError("semantic" if exc.stage == "semantic" else stage, exc.reason, raw) from exc

    steps_raw = d.get("steps", [])
    located: list[LocatedStep] = []
    for spec, entry in zip(stage_a_plan.steps, steps_raw):
        point, point_status, reason_codes = _parse_located_fields(
            stage, raw, spec.step_id, entry, coord_norm_max
        )
        located.append(
            LocatedStep(
                spec=spec,
                point_yx_norm1000=point,
                point_status=point_status,
                reason_codes=reason_codes,
            )
        )
    return stage_a_plan, tuple(located)


# ---------------------------------------------------------------------------
# Grasp affordance region (prompts/grasp_affordance.txt, ZeroDex App. D.2
# "Grasp Affordance" adapted single-view). Same localized/unavailable +
# reason_codes shape as LocatedStep, but for a 2D bounding box instead of a
# single point -- this is the region eq. 9 (B_v) asks for, single-view
# (M=1) since this workspace has one wrist camera, not the paper's
# multi-view rig. See src/mpg/affordance_region.py for turning this box
# into the 3D task_region_mask src/mpg/grasp_candidates.py expects.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AffordanceRegion:
    """One VLM response to prompts/grasp_affordance.txt.

    bbox_yx_norm1000 is (y1, x1, y2, x2) with y1 < y2 and x1 < x2, or None
    iff status is UNAVAILABLE (rule 5: typed abstention, never a guessed
    box)."""

    status: PointStatus
    description: str | None
    point_yx_norm1000: tuple[float, float] | None
    bbox_yx_norm1000: tuple[float, float, float, float] | None
    reason_codes: list[str]

    def to_json(self) -> dict:
        return {
            "status": self.status.value,
            "affordance_description": self.description,
            "affordance_point_yx_norm1000": (
                list(self.point_yx_norm1000) if self.point_yx_norm1000 is not None else None
            ),
            "affordance_bbox_yx_norm1000": (
                list(self.bbox_yx_norm1000) if self.bbox_yx_norm1000 is not None else None
            ),
            "reason_codes": list(self.reason_codes),
        }


def _validate_bbox(
    stage: str, raw: str, value: Any, coord_norm_max: int
) -> tuple[float, float, float, float]:
    if (
        not isinstance(value, list)
        or len(value) != 4
        or any(isinstance(v, bool) for v in value)
        or not all(isinstance(v, (int, float)) for v in value)
    ):
        raise ParseError(stage, "affordance_bbox_yx_norm1000 must be [y1, x1, y2, x2] numbers", raw)
    y1, x1, y2, x2 = (float(v) for v in value)
    if not all(math.isfinite(v) for v in (y1, x1, y2, x2)):
        raise ParseError(stage, "affordance_bbox_yx_norm1000 has NaN/Inf", raw)
    if not all(0 <= v <= coord_norm_max for v in (y1, x1, y2, x2)):
        raise ParseError(
            stage, f"affordance_bbox_yx_norm1000={value} out of [0, {coord_norm_max}]", raw
        )
    if not (y1 < y2 and x1 < x2):
        raise ParseError(
            stage,
            f"affordance_bbox_yx_norm1000={value} must have y1<y2 and x1<x2 "
            "(top-left corner then bottom-right corner)",
            raw,
        )
    return (y1, x1, y2, x2)


def parse_grasp_affordance_response(
    raw_text: str, *, coord_norm_max: int = DEFAULT_COORD_NORM_MAX
) -> AffordanceRegion:
    stage = "grasp_affordance"
    raw = raw_text
    text = strip_code_fence(raw_text)
    d = parse_json_strict(stage, text)
    if not isinstance(d, dict):
        raise ParseError(stage, "top-level response must be a JSON object", raw)

    status_raw = d.get("status")
    try:
        status = PointStatus(status_raw)
    except ValueError:
        raise ParseError(stage, f"invalid status {status_raw!r}", raw) from None

    reason_codes = _check_reason_codes(stage, raw, d.get("reason_codes", []))
    description_raw = d.get("affordance_description")
    point_raw = d.get("affordance_point_yx_norm1000")
    bbox_raw = d.get("affordance_bbox_yx_norm1000")

    if status is PointStatus.LOCALIZED:
        description = _require_str(stage, raw, d, "affordance_description")
        if point_raw is None:
            raise ParseError(stage, "LOCALIZED requires affordance_point_yx_norm1000", raw)
        point = _validate_point(stage, raw, "grasp_affordance", point_raw, coord_norm_max)
        if bbox_raw is None:
            raise ParseError(stage, "LOCALIZED requires affordance_bbox_yx_norm1000", raw)
        bbox = _validate_bbox(stage, raw, bbox_raw, coord_norm_max)
    else:
        if description_raw is not None or point_raw is not None or bbox_raw is not None:
            raise ParseError(
                stage,
                "UNAVAILABLE must have affordance_description/point/bbox all null",
                raw,
            )
        if not reason_codes:
            raise ParseError(stage, "UNAVAILABLE requires reason_codes", raw)
        description, point, bbox = None, None, None

    return AffordanceRegion(
        status=status, description=description, point_yx_norm1000=point,
        bbox_yx_norm1000=bbox, reason_codes=reason_codes,
    )


@dataclass(frozen=True)
class Meta:
    latency_s: float
    n_calls: int
    parse_retries: int

    def to_json(self) -> dict:
        return {
            "latency_s": self.latency_s,
            "n_calls": self.n_calls,
            "parse_retries": self.parse_retries,
        }


@dataclass(frozen=True)
class Plan:
    """The merged Plan JSON v0 (AGENTS.md Sec 2, Phase 2 'Definition of
    done' artifact): one parsed, validated plan per scene."""

    scene_id: str
    instruction: str
    backend: str
    stage_a: StageAPlan
    steps: tuple[LocatedStep, ...]
    meta: Meta

    def to_json(self) -> dict:
        return {
            "scene_id": self.scene_id,
            "instruction": self.instruction,
            "backend": self.backend,
            "stage_a": self.stage_a.to_json(),
            "steps": [s.to_json() for s in self.steps],
            "meta": self.meta.to_json(),
        }


def merge_plan(
    *,
    scene_id: str,
    instruction: str,
    backend: str,
    stage_a_plan: StageAPlan,
    located_steps: tuple[LocatedStep, ...],
    meta: Meta,
) -> Plan:
    a_ids = [s.step_id for s in stage_a_plan.steps]
    b_ids = [s.spec.step_id for s in located_steps]
    if a_ids != b_ids:
        raise ValueError(f"Stage A/B step_id order mismatch: {a_ids} vs {b_ids}")
    return Plan(
        scene_id=scene_id,
        instruction=instruction,
        backend=backend,
        stage_a=stage_a_plan,
        steps=located_steps,
        meta=meta,
    )


# ---------------------------------------------------------------------------
# Shared parser primitives (prompt_strategy.md Sec 4)
# ---------------------------------------------------------------------------

_FENCE_RE = re.compile(r"^\s*```(?:json)?\s*\n(.*?)\n?\s*```\s*$", re.DOTALL)


def strip_code_fence(text: str) -> str:
    """Strip at most one layer of Markdown code fence. Does not attempt to
    repair malformed fences or unrelated wrapping text — a model response
    that needs more than that is a format failure, not a parsing puzzle."""
    match = _FENCE_RE.match(text)
    return match.group(1) if match else text


def parse_json_strict(stage: str, text: str) -> Any:
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise ParseError(stage, f"invalid JSON: {exc}", text) from exc


# ---------------------------------------------------------------------------
# Coordinate adapter (AGENTS.md Sec 0.1 item 4: never index W or H)
# ---------------------------------------------------------------------------


def pixel_from_norm1000(
    point_yx_norm1000: tuple[float, float],
    *,
    width: int,
    height: int,
    adapter: str = "new",
    coord_norm_max: int = DEFAULT_COORD_NORM_MAX,
) -> tuple[float, float]:
    """Convert a normalized [y, x] point to pixel (x_px, y_px).

    adapter="new" (this project's contract): endpoint 1000 maps to (W-1)/(H-1),
    i.e. the center of the last pixel — never W/H themselves.
    adapter="legacy": reproduces the existing gemini_client.py behavior,
    x*(W)/1000, y*(H)/1000, for faithful baseline comparison only.
    """
    y_norm, x_norm = point_yx_norm1000
    if width <= 0 or height <= 0:
        raise ValueError(f"width/height must be positive, got {width}x{height}")
    if adapter == "new":
        x_scale, y_scale = (width - 1), (height - 1)
    elif adapter == "legacy":
        x_scale, y_scale = width, height
    else:
        raise ValueError(f"unknown adapter {adapter!r}")
    x_px = x_norm / coord_norm_max * x_scale
    y_px = y_norm / coord_norm_max * y_scale
    return (x_px, y_px)
