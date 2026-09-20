"""Ties backends + prompts + schema together: build a request, parse the
response, retry exactly once on a format failure, and give the caller
everything a run's requests.jsonl / failures.jsonl needs to stay
traceable (AGENTS.md rule 15: no false-green, every failure keeps its
raw text and the stage it failed at).

Parser pipeline per docs/prompt_strategy.md Sec 4: raw -> strip one fence
-> strict JSON -> schema -> semantic consistency -> coordinate validation
(all inside src/mpg/schema.py already). This module adds exactly one
thing on top: the retry-with-format-reminder policy, and only for that —
"format failures get exactly one retry with a format reminder; a correct
abstention is never retried" (rule 17 / prompt_strategy.md Sec 6). A
transport failure (timeout, HTTP error) is not caught here and is not
treated as a format failure or a model abstention.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from PIL import Image

from .schema import (
    DEFAULT_COORD_NORM_MAX,
    AffordanceRegion,
    LocatedStep,
    ParseError,
    PointStatus,
    ScenarioPlan,
    StageAPlan,
    StepType,
    parse_grasp_affordance_response,
    parse_single_call_response,
    parse_stage_0_response,
    parse_stage_b_response,
)
from .vlm.base import BaseBackend

PROMPTS_DIR = Path(__file__).resolve().parents[2] / "prompts"
_HEADER_SEPARATOR = "\n---\n\n"


class GroundingFailure(RuntimeError):
    """Raised when a stage exhausts its format retries. Carries everything
    needed to write one failures.jsonl row without re-deriving it."""

    def __init__(self, stage: str, last_error: ParseError, raw_responses: list[str], n_calls: int):
        super().__init__(f"{stage}: exhausted retries, last error: {last_error}")
        self.stage = stage
        self.last_error = last_error
        self.raw_responses = raw_responses
        self.n_calls = n_calls


@dataclass
class CallInfo:
    n_calls: int = 0
    cache_hits: int = 0
    logical_queries: int = 0
    parse_failures: int = 0
    parse_retries: int = 0
    latency_s: float = 0.0
    raw_responses: list[str] = field(default_factory=list)


def load_prompt_body(prompt_path: Path) -> str:
    """Strip the human-facing provenance header (everything before the
    first '---' separator) — that header documents lineage for reviewers,
    it is not meant to be sent to a model."""
    text = prompt_path.read_text()
    if _HEADER_SEPARATOR in text:
        return text.split(_HEADER_SEPARATOR, 1)[1]
    return text


def _format_reminder(original_prompt: str, error: ParseError) -> str:
    # prompt_strategy.md Sec 4: resend a complete, self-contained request
    # with the schema restated — never "this point looks wrong, retry" as
    # if it were the model's fault for a legitimate judgment.
    return (
        original_prompt
        + "\n\n---\n"
        + "Your previous response FAILED TO PARSE with this error:\n"
        + f"[{error.stage}] {error.reason}\n"
        + "Return ONLY corrected JSON that exactly matches the schema given "
        + "above. Do not include commentary, your previous response, or "
        + "markdown fences."
    )


def run_stage_0(
    backend: BaseBackend,
    image: Image.Image,
    goal: str,
    *,
    prompt_path: Path = PROMPTS_DIR / "stage_0_planner.txt",
    max_retries: int = 1,
    replicate_id: str = "",
) -> tuple[ScenarioPlan, CallInfo]:
    """Stage 0 (ZeroDex App. D.2 "Long-horizon Planner", adapted): break an
    underspecified instruction into a concrete, single-object atomic task
    naming the actual objects/colors in view. Same retry-once-on-format-
    failure policy as run_stage_a (prompt_strategy.md Sec 6)."""
    base_prompt = load_prompt_body(prompt_path).replace("{goal}", goal)
    info = CallInfo()
    prompt = base_prompt
    last_error: ParseError | None = None

    for attempt in range(max_retries + 1):
        result = backend.query(image, prompt, replicate_id=f"{replicate_id}:s0_{attempt}")
        info.logical_queries += 1
        info.cache_hits += int(result.cache_hit)
        info.n_calls += int(not result.cache_hit)
        info.latency_s += result.latency_s
        info.raw_responses.append(result.raw_text)
        try:
            plan = parse_stage_0_response(result.raw_text)
            return plan, info
        except ParseError as exc:
            last_error = exc
            info.parse_failures += 1
            if attempt < max_retries:
                info.parse_retries += 1
            prompt = _format_reminder(base_prompt, exc)

    assert last_error is not None
    raise GroundingFailure("stage_0", last_error, info.raw_responses, info.n_calls)


def run_stage_a(
    backend: BaseBackend,
    image: Image.Image,
    instruction: str,
    *,
    prompt_path: Path = PROMPTS_DIR / "stage_a.txt",
    max_retries: int = 1,
    replicate_id: str = "",
) -> tuple[StageAPlan, CallInfo]:
    base_prompt = load_prompt_body(prompt_path).replace("{instruction}", instruction)
    info = CallInfo()
    prompt = base_prompt
    last_error: ParseError | None = None

    for attempt in range(max_retries + 1):
        result = backend.query(image, prompt, replicate_id=f"{replicate_id}:a{attempt}")
        info.logical_queries += 1
        info.cache_hits += int(result.cache_hit)
        info.n_calls += int(not result.cache_hit)
        info.latency_s += result.latency_s
        info.raw_responses.append(result.raw_text)
        try:
            plan = StageAPlan.from_json_text(result.raw_text)
            return plan, info
        except ParseError as exc:
            last_error = exc
            info.parse_failures += 1
            if exc.stage == "semantic":
                break
            if attempt < max_retries:
                info.parse_retries += 1
            prompt = _format_reminder(base_prompt, exc)

    assert last_error is not None
    raise GroundingFailure("stage_a", last_error, info.raw_responses, info.n_calls)


def _render_stage_b_prompt(prompt_path: Path, stage_a_plan: StageAPlan) -> str:
    body = load_prompt_body(prompt_path)
    plan_steps_json = json.dumps([s.to_json() for s in stage_a_plan.steps], indent=2)
    return (
        body.replace("{target_name}", stage_a_plan.target.name if stage_a_plan.target else "null")
        .replace(
            "{destination_name}",
            stage_a_plan.destination.name if stage_a_plan.destination else "null",
        )
        .replace("{tool_name}", stage_a_plan.tool.name if stage_a_plan.tool else "null")
        .replace("{plan_steps_json}", plan_steps_json)
    )


def run_stage_b(
    backend: BaseBackend,
    image: Image.Image,
    stage_a_plan: StageAPlan,
    *,
    prompt_path: Path = PROMPTS_DIR / "stage_b.txt",
    max_retries: int = 1,
    coord_norm_max: int = DEFAULT_COORD_NORM_MAX,
    replicate_id: str = "",
) -> tuple[tuple[LocatedStep, ...], CallInfo]:
    base_prompt = _render_stage_b_prompt(prompt_path, stage_a_plan)
    info = CallInfo()
    prompt = base_prompt
    last_error: ParseError | None = None

    for attempt in range(max_retries + 1):
        result = backend.query(image, prompt, replicate_id=f"{replicate_id}:b{attempt}")
        info.logical_queries += 1
        info.cache_hits += int(result.cache_hit)
        info.n_calls += int(not result.cache_hit)
        info.latency_s += result.latency_s
        info.raw_responses.append(result.raw_text)
        try:
            located = parse_stage_b_response(
                result.raw_text, stage_a_plan, coord_norm_max=coord_norm_max
            )
            return located, info
        except ParseError as exc:
            last_error = exc
            info.parse_failures += 1
            if exc.stage == "semantic":
                break
            if attempt < max_retries:
                info.parse_retries += 1
            prompt = _format_reminder(base_prompt, exc)

    assert last_error is not None
    raise GroundingFailure("stage_b", last_error, info.raw_responses, info.n_calls)


class NoGraspStepError(ValueError):
    """No StepType.GRASP entry in located_steps (or its stage_b point was not
    LOCALIZED), so there is nothing to derive a grasp_hint from."""


def derive_grasp_hint(located_steps: tuple[LocatedStep, ...], *, instruction: str) -> tuple[str, str]:
    """Bridge from stage_a/stage_b's semantic-point pipeline to
    run_grasp_affordance()'s (grasp_object, grasp_hint) arguments, so the affordance
    call is conditioned on what stage_b already found instead of the caller
    hand-writing those two strings independently and possibly inconsistently with
    them. This is the piece ZeroDex App. D.2 eq. 9's l''' (the affordance prompt,
    described as built from the earlier-found grasp point's own description) was
    always meant to use; before this function existed, every caller in this
    repository invented its own grasp_object/grasp_hint strings from scratch
    (research/scripts/s4_live_affordance_smoke_test.py hardcoded "red cube" / "pick
    up the red cube" as plain literals, unconnected to any stage_a/stage_b call it
    could have made first) -- see docs/progress/grasp_motion_progress_report.md's
    2026-09-20 revision note for that audit finding.

    grasp_object comes from the GRASP step's own `desc` (e.g. "beige bar handle" --
    already a concrete visual description stage_a wrote, the same kind of string a
    caller would otherwise have typed by hand). grasp_hint combines the outer task
    instruction with that step's `geometric_meaning` so the affordance prompt knows
    both *what* to grasp and *why* (which region matters for THIS task), matching
    eq. 9's intent that the affordance region is task-conditioned, not just
    object-conditioned.

    Raises NoGraspStepError if there is no GRASP step, or its stage_b point never
    reached LOCALIZED (e.g. the object was judged not visible) -- never silently
    invents a hint for something stage_b itself could not find.
    """
    grasp_step = next((s for s in located_steps if s.spec.type == StepType.GRASP), None)
    if grasp_step is None:
        raise NoGraspStepError("located_steps has no StepType.GRASP entry")
    if grasp_step.point_status != PointStatus.LOCALIZED:
        raise NoGraspStepError(
            f"GRASP step {grasp_step.spec.step_id!r} was not LOCALIZED "
            f"(status={grasp_step.point_status.value}, reason_codes={grasp_step.reason_codes})"
        )
    grasp_object = grasp_step.spec.desc
    grasp_hint = f"{instruction} -- grasp target: {grasp_step.spec.geometric_meaning}"
    return grasp_object, grasp_hint


def run_grasp_affordance(
    backend: BaseBackend,
    image: Image.Image,
    *,
    grasp_object: str,
    grasp_hint: str,
    prompt_path: Path = PROMPTS_DIR / "grasp_affordance.txt",
    max_retries: int = 1,
    coord_norm_max: int = DEFAULT_COORD_NORM_MAX,
    replicate_id: str = "",
) -> tuple[AffordanceRegion, CallInfo]:
    """ZeroDex App. D.2 "Grasp Affordance" (eq. 9's B_v), single-view: ask
    the VLM to mark the contact region for grasp_object as a bounding box,
    not a point, explicitly excluding the object's functional tip. Same
    retry-once-on-format-failure policy as the other stages; a correct
    "unavailable" (object not visible/too occluded/no safe region) is
    never retried, matching run_stage_a/run_stage_b.

    grasp_object/grasp_hint are plain strings, typically already known
    from an earlier grounding stage (e.g. StageAPlan.target.name and the
    scenario/task instruction) -- this function does not itself decide
    what to grasp, only where on it is safe to touch.
    """
    base_prompt = (
        load_prompt_body(prompt_path)
        .replace("{grasp_object}", grasp_object)
        .replace("{grasp_hint}", grasp_hint)
    )
    info = CallInfo()
    prompt = base_prompt
    last_error: ParseError | None = None

    for attempt in range(max_retries + 1):
        result = backend.query(image, prompt, replicate_id=f"{replicate_id}:aff{attempt}")
        info.logical_queries += 1
        info.cache_hits += int(result.cache_hit)
        info.n_calls += int(not result.cache_hit)
        info.latency_s += result.latency_s
        info.raw_responses.append(result.raw_text)
        try:
            region = parse_grasp_affordance_response(result.raw_text, coord_norm_max=coord_norm_max)
            return region, info
        except ParseError as exc:
            last_error = exc
            info.parse_failures += 1
            if attempt < max_retries:
                info.parse_retries += 1
            prompt = _format_reminder(base_prompt, exc)

    assert last_error is not None
    raise GroundingFailure("grasp_affordance", last_error, info.raw_responses, info.n_calls)


def run_single_call(
    backend: BaseBackend,
    image: Image.Image,
    instruction: str,
    *,
    prompt_path: Path = PROMPTS_DIR / "single_call.txt",
    max_retries: int = 1,
    coord_norm_max: int = DEFAULT_COORD_NORM_MAX,
    replicate_id: str = "",
) -> tuple[StageAPlan, tuple[LocatedStep, ...], CallInfo]:
    base_prompt = load_prompt_body(prompt_path).replace("{instruction}", instruction)
    info = CallInfo()
    prompt = base_prompt
    last_error: ParseError | None = None

    for attempt in range(max_retries + 1):
        result = backend.query(image, prompt, replicate_id=f"{replicate_id}:sc{attempt}")
        info.logical_queries += 1
        info.cache_hits += int(result.cache_hit)
        info.n_calls += int(not result.cache_hit)
        info.latency_s += result.latency_s
        info.raw_responses.append(result.raw_text)
        try:
            stage_a_plan, located = parse_single_call_response(
                result.raw_text, coord_norm_max=coord_norm_max
            )
            return stage_a_plan, located, info
        except ParseError as exc:
            last_error = exc
            info.parse_failures += 1
            if exc.stage == "semantic":
                break
            if attempt < max_retries:
                info.parse_retries += 1
            prompt = _format_reminder(base_prompt, exc)

    assert last_error is not None
    raise GroundingFailure("single_call", last_error, info.raw_responses, info.n_calls)
