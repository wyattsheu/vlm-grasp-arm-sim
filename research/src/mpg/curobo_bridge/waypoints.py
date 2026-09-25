"""Waypoint sequencing for reactive.py's --waypoints mode: advance to the next
target only once the current one is reached (within tolerance) and held for
its hold_s, with a per-waypoint timeout so a single unreachable target can't
hang a live demo forever.

Deliberately pure Python + numpy + PyYAML -- no curobo, ROS, or Isaac import
-- so research/tests/test_waypoint_sequencer.py runs under
env_robot129_research and exercises the actual state machine reactive.py
uses, not a reimplementation of it. reactive.py is the only caller that feeds
this real position/orientation error each control tick.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from mpg.curobo_bridge.frames import NAMED_DIRECTION_ALIASES, approach_direction_to_quat_xyzw

DEFAULT_HOLD_S = 1.0
DEFAULT_TIMEOUT_S = 15.0


@dataclass(frozen=True)
class Waypoint:
    name: str
    position_m: np.ndarray       # (3,)
    quaternion_xyzw: np.ndarray  # (4,)
    hold_s: float = DEFAULT_HOLD_S
    timeout_s: float = DEFAULT_TIMEOUT_S
    # Optional prevalidated IK solution.  ab_poses.yaml contains this because the
    # pose selector already solved and FK-checked A/B; re-solving the same pose at
    # demo startup is both wasteful and nondeterministic.
    joint_values_rad: np.ndarray | None = None  # (6,)


@dataclass
class WaypointStatus:
    """One update() tick's worth of reporting -- what reactive.py publishes to
    /robot129_sim/reactive/status and what run_demo_scenario.sh's report.json
    is built from.
    """
    index: int
    name: str
    phase: str  # "approaching" | "holding" | "done"
    position_error_m: float
    orientation_error_rad: float
    time_in_phase_s: float
    advanced: bool     # True on the tick a transition to the next waypoint happened
    timed_out: bool    # True if that transition was forced by a timeout, not tolerance
    all_done: bool


def _resolve_orientation(entry: dict) -> np.ndarray:
    """A waypoint entry gives orientation either directly (`quaternion_xyzw`)
    or as a named `approach` direction (+ optional `closing_hint`), matching
    research/scripts/select_demo_poses.py's convention -- see
    mpg.curobo_bridge.frames.approach_direction_to_quat_xyzw.
    """
    if "quaternion_xyzw" in entry:
        return np.asarray(entry["quaternion_xyzw"], dtype=float)
    if "approach" not in entry:
        raise ValueError(f"waypoint {entry.get('name', '?')!r} needs either 'quaternion_xyzw' or 'approach'")
    approach = entry["approach"]
    approach_world = NAMED_DIRECTION_ALIASES.get(approach)
    if approach_world is None:
        raise ValueError(f"unknown approach direction {approach!r}, expected one of {sorted(NAMED_DIRECTION_ALIASES)}")
    closing_hint = entry.get("closing_hint")
    if closing_hint is None:
        closing_hint_world = np.array([0.0, 0.0, 1.0])
    else:
        closing_hint_world = NAMED_DIRECTION_ALIASES.get(closing_hint, np.asarray(closing_hint, dtype=float))
    return approach_direction_to_quat_xyzw(approach_world, closing_hint_world)


def _optional_joint_values(entry: dict) -> np.ndarray | None:
    values = entry.get("joint_values_rad")
    if values is None:
        return None
    joints = np.asarray(values, dtype=float)
    if joints.shape != (6,):
        raise ValueError(f"waypoint {entry.get('name', '?')!r} joint_values_rad must contain 6 values")
    return joints


def load_waypoints_from_yaml(path: Path | str) -> list[Waypoint]:
    """Parse a demo waypoint file. Expected shape:

        waypoints:
          - name: A
            position_m: [0.25, 0.35, 0.58]
            approach: up          # or: quaternion_xyzw: [x, y, z, w]
            hold_s: 1.5
            timeout_s: 20.0
          - name: B
            position_m: [0.05, 0.35, 0.35]
            approach: left
    """
    doc = yaml.safe_load(Path(path).read_text())
    entries = doc["waypoints"] if isinstance(doc, dict) else doc
    waypoints = []
    for entry in entries:
        waypoints.append(Waypoint(
            name=entry["name"],
            position_m=np.asarray(entry["position_m"], dtype=float),
            quaternion_xyzw=_resolve_orientation(entry),
            hold_s=float(entry.get("hold_s", DEFAULT_HOLD_S)),
            timeout_s=float(entry.get("timeout_s", DEFAULT_TIMEOUT_S)),
            joint_values_rad=_optional_joint_values(entry),
        ))
    if not waypoints:
        raise ValueError(f"{path}: no waypoints found")
    return waypoints


def load_waypoints_from_ab_poses(path: Path | str, hold_s: float = DEFAULT_HOLD_S, timeout_s: float = DEFAULT_TIMEOUT_S) -> list[Waypoint]:
    """Build the [A, B] waypoint pair directly from
    research/scripts/select_demo_poses.py's output (demo_ab_poses_v1 schema) --
    the p2p_cone scenario and tools/send_joint_cmd.py's --sequence both read
    the SAME file this way, so "A points up, B points left" is asserted once,
    not copy-pasted into two configs that could drift apart.
    """
    doc = yaml.safe_load(Path(path).read_text())
    if doc.get("schema_version") != "demo_ab_poses_v1":
        raise ValueError(f"{path}: expected schema_version demo_ab_poses_v1, got {doc.get('schema_version')!r}")
    points = doc["points"]
    return [
        Waypoint(
            name=points["A"]["name"],
            position_m=np.asarray(points["A"]["position_m"], dtype=float),
            quaternion_xyzw=np.asarray(points["A"]["quaternion_xyzw"], dtype=float),
            hold_s=hold_s, timeout_s=timeout_s,
            joint_values_rad=_optional_joint_values(points["A"]),
        ),
        Waypoint(
            name=points["B"]["name"],
            position_m=np.asarray(points["B"]["position_m"], dtype=float),
            quaternion_xyzw=np.asarray(points["B"]["quaternion_xyzw"], dtype=float),
            hold_s=hold_s, timeout_s=timeout_s,
            joint_values_rad=_optional_joint_values(points["B"]),
        ),
    ]


class WaypointSequencer:
    """Drives one waypoint at a time. Call update() once per control tick with
    the CURRENT measured position/orientation error against `.current`'s
    target (the caller -- reactive.py -- owns computing that error via FK/IK,
    this class only owns the state machine: approaching -> holding -> next).
    """

    def __init__(
        self,
        waypoints: list[Waypoint],
        position_tolerance_m: float = 0.01,
        orientation_tolerance_rad: float = 0.0872665,  # ~5 deg
        loop: bool = False,
    ):
        if not waypoints:
            raise ValueError("waypoints must be non-empty")
        self._waypoints = list(waypoints)
        self._position_tolerance_m = position_tolerance_m
        self._orientation_tolerance_rad = orientation_tolerance_rad
        self._loop = loop
        self._index = 0
        self._approach_start_s: float | None = None
        self._hold_start_s: float | None = None
        self._done = False
        self._history: list[dict] = []

    @property
    def current(self) -> Waypoint:
        return self._waypoints[self._index]

    @property
    def done(self) -> bool:
        return self._done

    @property
    def history(self) -> list[dict]:
        """One entry per waypoint actually reached (by tolerance or timeout),
        in arrival order -- the source for reactive_report.json's per-waypoint
        table (see docs/dev_guide_paper_core_and_dashboard_plan.md Phase 2).
        """
        return list(self._history)

    def update(self, position_error_m: float, orientation_error_rad: float, now_s: float) -> WaypointStatus:
        if self._done:
            last = self._waypoints[-1]
            return WaypointStatus(
                index=len(self._waypoints) - 1, name=last.name, phase="done",
                position_error_m=position_error_m, orientation_error_rad=orientation_error_rad,
                time_in_phase_s=0.0, advanced=False, timed_out=False, all_done=True,
            )

        if self._approach_start_s is None:
            self._approach_start_s = now_s

        wp = self.current
        within_tolerance = position_error_m <= self._position_tolerance_m and orientation_error_rad <= self._orientation_tolerance_rad
        advanced = False
        timed_out = False
        phase = "approaching"

        if within_tolerance:
            phase = "holding"
            if self._hold_start_s is None:
                self._hold_start_s = now_s
            elif now_s - self._hold_start_s >= wp.hold_s:
                advanced = self._advance(now_s, timed_out=False, position_error_m=position_error_m, orientation_error_rad=orientation_error_rad)
        else:
            self._hold_start_s = None  # tolerance lost mid-hold -- the hold timer must restart, not resume
            if now_s - self._approach_start_s > wp.timeout_s:
                advanced = self._advance(now_s, timed_out=True, position_error_m=position_error_m, orientation_error_rad=orientation_error_rad)
                timed_out = True

        reported_index = self._index if not self._done else len(self._waypoints) - 1
        reported_name = wp.name if not self._done else self._waypoints[-1].name
        reported_phase = "done" if self._done else phase
        phase_start = self._hold_start_s if (phase == "holding" and self._hold_start_s is not None) else self._approach_start_s
        time_in_phase = 0.0 if self._done else now_s - phase_start

        return WaypointStatus(
            index=reported_index, name=reported_name, phase=reported_phase,
            position_error_m=position_error_m, orientation_error_rad=orientation_error_rad,
            time_in_phase_s=time_in_phase, advanced=advanced, timed_out=timed_out, all_done=self._done,
        )

    def _advance(self, now_s: float, timed_out: bool, position_error_m: float, orientation_error_rad: float) -> bool:
        self._history.append({
            "index": self._index, "name": self._waypoints[self._index].name,
            "arrived_at_s": now_s, "timed_out": timed_out,
            "final_position_error_m": position_error_m, "final_orientation_error_rad": orientation_error_rad,
        })
        if self._index + 1 < len(self._waypoints):
            self._index += 1
        elif self._loop:
            self._index = 0
        else:
            self._done = True
            return True
        self._approach_start_s = now_s
        self._hold_start_s = None
        return True
