"""Offline multi-point grounding research utilities."""

from .scene_bundle import validate_scene
from .schema import (
    LocatedStep,
    Meta,
    ParseError,
    Plan,
    PlanStepSpec,
    Role,
    StageAPlan,
    merge_plan,
    parse_single_call_response,
    parse_stage_b_response,
    pixel_from_norm1000,
    strip_code_fence,
)

__all__ = [
    "validate_scene",
    "LocatedStep",
    "Meta",
    "ParseError",
    "Plan",
    "PlanStepSpec",
    "Role",
    "StageAPlan",
    "merge_plan",
    "parse_single_call_response",
    "parse_stage_b_response",
    "pixel_from_norm1000",
    "strip_code_fence",
]
