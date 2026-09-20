"""JSON contract for GraspCandidate export/import (grasp_motion_research_plan_20260918.md
§8). Kept separate from grasp_candidates.py's generation logic so a future learned or
sampled candidate source (S6) can produce the same contract without depending on the
geometric generator's internals.
"""
from __future__ import annotations

import json
from pathlib import Path

from .grasp_candidates import GraspCandidate, REJECTION_REASONS

SCHEMA_VERSION = "grasp_candidate_v0"

_REQUIRED_FIELDS = {
    "schema_version", "scene_id", "candidate_id", "source", "frame_id",
    "tcp_pose", "pregrasp_pose", "approach_axis_frame", "approach_axis",
    "opening_width_m", "rejection_reasons",
}


class GraspContractError(ValueError):
    pass


def candidate_to_dict(candidate: GraspCandidate, *, scene_id: str, object_id: str | None = None) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "scene_id": scene_id,
        "object_id": object_id,
        "candidate_id": candidate.candidate_id,
        "source": candidate.source,
        "frame_id": candidate.frame_id,
        "tcp_frame": "pinch_center",
        "tcp_pose": {
            "position_m": list(candidate.tcp_position_m),
            "quaternion_xyzw": list(candidate.tcp_quaternion_xyzw),
        },
        "pregrasp_pose": {
            "position_m": list(candidate.pregrasp_position_m),
            "quaternion_xyzw": list(candidate.pregrasp_quaternion_xyzw),
        },
        "gripper_base_pose": {
            "position_m": list(candidate.gripper_base_position_m),
            "quaternion_xyzw": list(candidate.tcp_quaternion_xyzw),
        },
        "approach_axis_frame": candidate.frame_id,
        "approach_axis": list(candidate.approach_axis_world),
        "yaw_rad": candidate.yaw_rad,
        "opening_width_m": candidate.opening_width_m,
        "contact_point_indices": list(candidate.contact_point_indices),
        "geometry_source": "points_xyz",
        "score": candidate.score,
        "accepted": candidate.accepted,
        "rejection_reasons": list(candidate.rejection_reasons),
    }


def validate_candidate_dict(d: dict) -> None:
    missing = _REQUIRED_FIELDS - d.keys()
    if missing:
        raise GraspContractError(f"missing required fields: {sorted(missing)}")
    for key in ("tcp_pose", "pregrasp_pose"):
        pose = d[key]
        pos = pose.get("position_m")
        quat = pose.get("quaternion_xyzw")
        if not (isinstance(pos, list) and len(pos) == 3):
            raise GraspContractError(f"{key}.position_m must have 3 elements")
        if not (isinstance(quat, list) and len(quat) == 4):
            raise GraspContractError(f"{key}.quaternion_xyzw must have 4 elements")
    unknown_reasons = set(d["rejection_reasons"]) - REJECTION_REASONS
    if unknown_reasons:
        raise GraspContractError(f"unknown rejection reasons: {sorted(unknown_reasons)}")
    if d["accepted"] and d["rejection_reasons"]:
        raise GraspContractError("accepted candidate must have no rejection_reasons")
    if not d["accepted"] and not d["rejection_reasons"]:
        raise GraspContractError("rejected candidate must carry at least one rejection reason")


def write_candidates_json(
    path: Path, candidates: list[GraspCandidate], *, scene_id: str, object_id: str | None = None
) -> None:
    rows = [candidate_to_dict(c, scene_id=scene_id, object_id=object_id) for c in candidates]
    for row in rows:
        validate_candidate_dict(row)
    Path(path).write_text(json.dumps({"schema_version": SCHEMA_VERSION, "scene_id": scene_id,
                                       "candidates": rows}, indent=2))


def read_candidates_json(path: Path) -> list[dict]:
    payload = json.loads(Path(path).read_text())
    rows = payload.get("candidates", [])
    for row in rows:
        validate_candidate_dict(row)
    return rows
