# Multi-Point VLM Grounding for Robot 129 (ZeroDex-style) — Research Plan & Codex Task Spec

> Owner: Wyatt (許懷仁) · Collaborator: Tuan "Fungi" (arm / executor side) · Supervisor: Do Huu Phu (杜有富)
> ACM Lab, NYCU · Target: demo on **Thursday 2026-09-17**
> Save this file as `AGENTS.md` at the root of `WORK_DIR` so it is loaded in every Codex session.

---

## 0. Configuration (paths discovered read-only on 2026-09-11)

```
WORK_DIR       = /home/wyattsheu/workspaces/next_arm_tesk     # the ONLY writable location
ROS_ENV_DIR    = /home/wyattsheu/workspaces/robotic_agent/mm_system/main_ws/__pycache__/codex_next_arm_env
ROBOT_REPOS    = [/home/wyattsheu/workspaces/robotic_agent/mm_system, /home/acm/robotic_agent/robotic_system, /home/wyattsheu/workspaces/mm_system, /home/wyattsheu/workspaces/robotic_agent]  # read-only; first two are live-mounted sources
LOCAL_VLM_CODE = /home/wyattsheu/workspaces/temp/production_client  # read-only
GEMINI_MODEL   = gemini-robotics-er-2-preview                     # verify against the code in Phase 0
GEMINI_API_KEY = read from environment variable only
HOST           = Jetson AGX Thor ("Robot 129"), shared account used by other lab members
```

User-granted exception (2026-09-11): `ROS_ENV_DIR` is the only writable path
outside WORK_DIR. It is already Git-ignored and visible in `mm_container` as
`/workspace/main_ws/__pycache__/codex_next_arm_env`. Use it only for the
isolated ROS environment, copied runner, and capture staging/data. Every other
path under `robotic_agent` remains read-only; never edit ignore rules or source.

If any `<FILL IN>` is still unfilled when you start, stop and ask Wyatt. Do not guess paths.

---

## 0.1 Planning status and verified amendments (2026-09-11)

Detailed execution documents: `docs/execution_playbook.md`, `docs/data_and_evaluation.md`, `docs/prompt_strategy.md`, and `docs/methods_and_references.md`. Use the detailed data/evaluation protocol for experimental budgets and one-variable controls. These documents are plans, not implemented tools or completed phases.

The user requested planning first, then authorized research and experiments. Phase 0 completed its safe read-only scope on 2026-09-11 and is awaiting the Phase 1 gate. Do not start capture, inference, implementation, or Phase 5 without the applicable phase gate. See `docs/research_plan.md`, `docs/00_system_recon.md`, and `docs/questions.md`.

All configured paths exist. Docker mount inspection identifies the first two ROBOT_REPOS entries as current live-mounted source trees; the exact installed build and which of two running decision containers handles commands remain UNVERIFIED. LOCAL_VLM_CODE is an older standalone client and differs from the current live-mounted integration. Do not execute scripts found there. Expanded tool permissions do not relax the WORK_DIR-only write rule or robot restrictions.

The following evidence-based corrections take precedence over conflicting assumptions later in this original task specification. Proposed algorithm changes below are design decisions for review, not permission to implement:

1. Existing local code uses OpenAI-compatible **vLLM**, with text-only Qwen3-VL-4B action/label extraction and Molmo2-4B visual pointing, not Ollama. Its `decide_task` returns pixel `[x,y]` or None, not a generic image+prompt query. Preserve this as the legacy baseline; a rich-plan adapter must be separate. Source: `LOCAL_VLM_CODE/local_pipeline_client.py:109,139,177,217`.
2. Existing Gemini model is ER 2, but the existing key variable is `GOOGLE_API_KEY`; new isolated code uses `GEMINI_API_KEY` as specified. Never read out, copy, or log secret values. Source: `mm_system/.../reasoning/gemini_client.py:89` and `mm_actions_node.py:55` (full paths in recon).
3. Existing depth uses an **11x11 mean**, range 0.1–3.0 m, not a median. New 7x7 median is an experimental change, not faithful reuse. Source: `mm_system/.../perception/utils.py:39,48,58`.
4. Keep native coordinates plus explicit adapters. For the new normalized endpoint convention use x*(W-1)/1000, y*(H-1)/1000; preserve W/H scaling in the legacy reproduction. Never index W or H. Report this adapter difference and isolate it from model/prompt comparisons.
5. Do not force a guessed coordinate for absent/occluded targets. Store null with a typed unavailable status; format failures and intentional abstention are separate. This intentionally differs from the original reference-view prompt's never-null rule and preserves the legacy client's absence signal.
6. Semantic WAYPOINT is only a proposal. Single-view depth is a surface measurement; derive transport height geometrically, record modified 3D projection separately, and check all transfer segments against observed geometry. Unseen space stays unknown. Point-cloud clearance is not full robot collision feasibility; IK stays NOT RUN until Phase 5 approval.
7. Distinguish target contact, object reference, and TCP. Do not apply the original release-height equation blindly; object-to-TCP offset, orientation, destination support/opening, and gripper envelope must be explicit. Existing camera-depth offset and base-Z offset are different transforms.
8. Final evaluation holds out scenes and counts every actual backend call. Consistency uses independent samples with replicate IDs, not repeated cache hits. At 12 scenes x 3 repeats x (2+2+1) Gemini calls = 180 before retries; reserve 20 and stop before 200. Local legacy two-stage calls count separately.
9. Local PDF has **21 pages** (pdfinfo verified). Sec 3.1 and Appendix D.2 have different primitive/reference details; see `refs/reading_notes.md`. The plan is ZeroDex-inspired, not a full reproduction. Papers' success rates are not results on our robot.

Original supplied specification is archived unchanged as `refs/original_task_spec.md`. Hardware, current services, deployment state, calibration accuracy, and research outcomes remain UNVERIFIED / NOT RUN unless separately evidenced.

## 1. Hard rules (read first, never violate)

These rules exist because the host is a shared robot computer. Other people's code and a physical arm are on it.

### 1.1 Filesystem sandbox
1. **Write only inside `WORK_DIR`.** Every file you create, modify, cache, or log goes under `WORK_DIR`. No exceptions.
2. **Everything outside `WORK_DIR` is read-only.** You may read code in `ROBOT_REPOS` and `LOCAL_VLM_CODE`. You may not edit, format, rename, delete, or create files there.
3. **No git operations outside `WORK_DIR`.** No `commit`, `checkout`, `switch`, `stash`, `reset`, `pull`, `push`, `clean`, or branch creation in other repos. You may run `git -C WORK_DIR init` and commit inside `WORK_DIR` only.
4. **Reusing existing code**: prefer importing read-only (add the repo path to `sys.path` at runtime, never install it). If you must copy a snippet, such as the current pointing prompt, put it under `WORK_DIR/vendor/` or `WORK_DIR/prompts/` with a header comment giving the source path, line range, and git commit hash.
5. **No system changes.** No `sudo`, `apt`, `snap`, `systemctl`, or edits to `~/.bashrc`, `~/.profile`, `~/.ssh`, crontab, `/etc`, or global config files.
6. **Python environment**: create `WORK_DIR/.venv` with `python3 -m venv --system-site-packages WORK_DIR/.venv`. System site-packages are needed for ROS 2 `rclpy`. Install packages only into this venv. Never `pip install` globally or with `--break-system-packages`.

### 1.2 Robot safety
7. **Never command robot motion.** This covers the Piper arm, gripper, and Kachaka base. Forbidden:
   - `ros2 topic pub`, `ros2 service call`, `ros2 action send_goal`
   - `ros2 launch` or `ros2 run` of existing packages
   - Piper SDK enable/motion calls
   - CAN interface commands (`ip link`, `candump` writes)
   - Kachaka API calls
8. **Allowed ROS 2 commands (read-only)**: `ros2 node list`, `ros2 topic list/info/hz`, `ros2 topic echo --once`, `ros2 run tf2_ros tf2_echo`, `ros2 param get`, and a subscriber-only capture script you write in `WORK_DIR`.
9. **No motion-capable code before Phase 5 approval.** Until Wyatt explicitly approves Phase 5, no script in `WORK_DIR` may contain publishers or clients to arm/gripper/base topics, services, or actions. Phase 5 is dry-run only: compute IK, never send it. Physical execution is done by humans (Wyatt/Tuan), not by you.
10. **Do not stop, restart, or kill any process you did not start.** This includes ROS 2 nodes, the camera driver, and the Ollama server.

### 1.3 Shared resources, secrets, cost
11. **Ollama**: do not `ollama pull`, `rm`, or change server settings without approval. Use whatever models are already present.
12. **Local inference**: keep GPU/CPU use modest. Run one inference at a time, and never loop local inference in the background unattended.
13. **Secrets**: read `GEMINI_API_KEY` from the environment. Never print it, log it, write it to a file, or commit it. If it is missing, ask.
14. **Gemini cost control**: cache every VLM response on disk (`WORK_DIR/cache/`, keyed by a hash of model + prompt + image). Log every real API call to `WORK_DIR/logs/vlm_calls.jsonl` with timestamp, model, and latency. Default cap is **200 real Gemini calls per phase**; ask before exceeding it.

### 1.4 Honesty and process
15. **No false-green.** Report something as working only if you actually ran it and the output file exists. Give the path. Anything not executed is marked `NOT RUN`. Anything inferred but not checked is marked `UNVERIFIED`.
16. **Trace claims about the existing system to `file:line`.** "The grasp uses depth median" must point to the exact line. If you cannot find it, say so.
17. **Change one variable at a time** in experiments. Do not change the model and the prompt in the same comparison.
18. **Approval gates**: at the end of every phase, stop, write the phase report (Section 7), and wait for Wyatt's go-ahead before starting the next phase.
19. **This document may contain mistakes.** Statements marked *(to verify)* are Wyatt's current understanding. If the code disagrees, the code wins. Record the discrepancy in `docs/questions.md`.

---

## 2. Background

### 2.1 Current system (to verify in Phase 0)
Robot 129 is a mobile manipulator:
- Jetson AGX Thor
- Piper 6-DoF arm with a parallel gripper (CAN bus)
- Intel RealSense D435i mounted eye-in-hand
- Kachaka mobile base
- ROS 2 stack

The current manipulation flow, roughly:

```
base moves to location A → RGB(-D) image → VLM returns ONE 2D point (object)
→ deproject with depth → camera→base transform → IK (roboticstoolbox ik_LM / QP servo) → grasp
base moves to location B → new image → VLM returns ONE 2D point (release place) → IK → release
```

- The VLM was Gemini Robotics-ER, via `gemini_client.py`. Gemini Robotics-ER 1.6 was shut down at the end of August 2026, so the model string should now be `gemini-robotics-er-2-preview`.
- Wyatt is finishing a replacement of Gemini pointing with a local VLM served by Ollama on the Thor. **That work is in progress. Do not modify it.**
- Files Wyatt has read before *(to verify)*: `grasp.py`, `gemini_client.py`, `piper_kinematic.py`, `mm_actions_node.py`, `decision_maker_node.py`, `scenario_library.py`.
- Repos *(to verify)*: `robotic_system` (Jimmy), `robotic_agent` (decision-team fork), `mm_system` (arm team).
- The same repos also contain teleoperation (`mujoco_teleop_sim.py`, `mink` IK) and X-VLA evaluation code belonging to other members. **Ignore and do not touch.**

### 2.2 Problem
One point per VLM call only supports simple pick/place. There is no transport waypoint, so there is no obstacle avoidance. Nothing is shared between the pick query and the place query. Extending to more complex or tool-based tasks requires richer, task-conditioned grounding.

### 2.3 Idea (from ZeroDex, arXiv 2606.19340)
ZeroDex decouples semantic reasoning (VLM) from execution (motion primitives).

- **Stage A — task grounding**: from the instruction and image, the VLM infers the mode (pick-and-place vs. tool-use), the target object, the tool, and the destination.
- **Stage B — primitive planning**: the VLM produces a primitive sequence, each step with a 2D keypoint and a semantic description:
  - pick-and-place: **grasp → waypoint → release**
  - tool-use: grasp (+ functional tip) → apply_action → release/hold
- It then lifts the keypoints to 3D using multi-view fusion, refines placement points for collisions, and executes.

**Hardware gap.** ZeroDex uses an xArm, an Inspire dexterous hand, and 4–6 fixed calibrated RGB cameras. We have a parallel gripper and **one** wrist-mounted RGB-D camera. Therefore:
- **We adopt**: Stage A/B prompting, the primitive plan structure, the prompt design rules, collision-aware refinement, and the evaluation metrics.
- **We replace**: multi-view triangulation with **single-view RGB-D lifting**. This is ZeroDex's own "RGB-D baseline".
- **We skip**: dexterous grasp generation (Sec 3.4, App. B), the Bag of Atomic Actions tool trajectories (Sec 3.3), and FoundationPose/FoundationStereo.
- **Future work (design only, no implementation now)**: *active multi-view*. Move the wrist camera to 2–3 arm poses; each camera pose is known from FK + hand-eye calibration. Then apply ZeroDex triangulation and ray voting.

### 2.4 Research questions for this sprint
- **RQ1.** Does a single structured VLM plan (grasp / waypoint / release) ground as accurately as the current one-point-per-call approach, while using fewer calls?
- **RQ2.** Can single-view RGB-D lifting, plus simple geometric rules and collision-aware refinement, produce 3D waypoint and release points that are valid (in free space, reachable) on our robot?
- **RQ3.** How do Gemini ER 2 and the local VLM compare on this richer prompt (accuracy, parse rate, latency)?

---

## 3. References (look these up yourself; if a link fails, search — never invent content)

### 3.1 Primary paper — read before Phase 2
**ZeroDex** — Kim et al., "Zero-Shot Long-Horizon Dexterous Manipulation via Multi-View 3D-Grounded VLM Reasoning", 2026.

Links:
- arXiv: https://arxiv.org/abs/2606.19340
- Project page: https://jlogkim.github.io/zerodex (check whether code has been released; as of 2026-09-11 none was found)
- Local copy: `WORK_DIR/refs/zerodex_2606.19340v2.pdf`

Reading guide (page numbers refer to the PDF):

| Section | Pages | Priority | Why |
|---|---|---|---|
| Fig. 1 + Sec 3 intro | p.3–4 | must | pipeline overview |
| Sec 3.1 Reference-Frame Grounding (Eq. 1–2) | p.4 | **must** | Stage A/B definition, primitive set, keypoint counts |
| App. D.2 prompts: Long-horizon Planner, Multi-view Roles and Plan Selector, Point Localization from a Fixed Plan | p.18–20 | **must** | prompt templates to adapt |
| Eq. 12 + App. D.1 collision-aware refinement | p.6, p.17 | must | ±4 cm neighborhood, 2 cm grid, vertical-first |
| Sec 4.3 metrics, Table 3, Fig. S1 | p.7–8, p.14 | must | evaluation design; RGB-D baseline vs. multi-view |
| Sec 3.2 multi-view lifting, App. D.1 fusion params | p.4–5, p.17 | later | future active multi-view |
| App. C.1 closed-loop verify/replan | p.16–17 | later | future work |
| Sec 3.3, 3.4, App. B | p.5–6, p.15–16 | skip | dexterous hand / tool trajectories, not applicable |

Prompt design rules to carry over from App. D.2 (adapted to single view):
- Ask the model to first describe what is visible and what is occluded, then reason. This forces evidence-grounded answers.
- Identify roles explicitly: TOOL (never the robot hand), TARGET, DESTINATION (may be null).
- Each plan step carries `desc` and a free-text `geometric_meaning`. **Forbid image-relative words** (left/right/top/bottom of the image).
- The waypoint must differ from the release point. It is a transit point chosen to avoid obstacles, with generous clearance.
- Never output null for points; give the best estimate. On parse failure, retry once with a format reminder.
- Coordinates are `[y, x]` normalized to 0–1000 (same convention as Gemini Robotics-ER).
- **Drop** the multi-view parts (per-view visibility, best-view selection, recipe selection).

**Note:** ZeroDex used `gemini-robotics-er-1.6-preview`, which is now retired. Its prompts may need retuning for ER 2.

### 3.2 VLM APIs
- Gemini Robotics-ER overview: https://ai.google.dev/gemini-api/docs/robotics-overview
- Gemini Robotics-ER spatial reasoning / pointing: https://ai.google.dev/gemini-api/docs/robotics-spatial
- Ollama: https://github.com/ollama/ollama (see the REST API docs; images are passed as base64)
- Before writing the local backend, read how `LOCAL_VLM_CODE` already calls the local model and reuse that interface.

### 3.3 Related methods (background; skim abstracts only if useful)
- MOKA — mark-based visual prompting: https://arxiv.org/abs/2403.03174
- ReKep — relational keypoint constraints: https://arxiv.org/abs/2409.01652
- PIVOT — iterative visual prompting: https://arxiv.org/abs/2402.07872
- RoboPoint — VLM spatial affordance points: https://arxiv.org/abs/2406.10721
- VoxPoser — 3D value maps from LLM/VLM: https://arxiv.org/abs/2307.05973
- Inner Monologue — closed-loop language feedback: https://arxiv.org/abs/2207.05608
- Gemini Robotics 1.5: https://arxiv.org/abs/2510.03342
- Qwen3-VL technical report: https://arxiv.org/abs/2511.21631
- Molmo / PixMo (open pointing VLM; Tuan mentioned it): https://arxiv.org/abs/2409.17146

### 3.4 Tools and libraries
- librealsense (see `rs2_deproject_pixel_to_point`): https://github.com/IntelRealSense/librealsense
- realsense-ros: https://github.com/IntelRealSense/realsense-ros
- ROS 2 docs, including tf2 tutorials (match the distro installed on the Thor): https://docs.ros.org/
- Open3D: https://www.open3d.org/docs/
- Robotics Toolbox for Python: https://github.com/petercorke/robotics-toolbox-python
- Piper SDK (read only, to understand the existing code): https://github.com/agilexrobotics/piper_sdk
- OpenCV (for future work: `triangulatePoints`, `calibrateHandEye`): https://docs.opencv.org/

---

## 4. Workspace layout (everything under `WORK_DIR`)

```
WORK_DIR/
├── AGENTS.md                 # this file
├── README.md                 # how to run each phase
├── refs/                     # ZeroDex PDF, reading notes
├── docs/
│   ├── 00_system_recon.md    # Phase 0 output
│   ├── 01_design.md          # schema, prompts, lifting rules
│   ├── 02_results.md         # Phase 4 output
│   ├── interface_contract.md # JSON contract for Tuan's executor
│   ├── future_active_multiview.md
│   ├── questions.md          # open questions and discrepancies for Wyatt
│   └── phase_reports/        # one report per phase
├── configs/default.yaml      # all thresholds and parameters (no magic numbers in code)
├── prompts/                  # stage_a.txt, stage_b.txt, single_call.txt, baseline_copied.txt (with provenance)
├── vendor/                   # copied snippets with provenance headers (avoid if import works)
├── src/mpg/                  # package: schema, vlm backends, grounding, lifting, refine, viz, eval
├── scripts/                  # capture_scene.py, run_grounding.py, run_eval.py, make_demo_assets.py
├── tests/                    # unit tests (schema parsing, coordinate conversion, deprojection)
├── data/scenes/<scene_id>/   # rgb.png, depth.png (16-bit, aligned), camera_info.json, tf.json,
│                             # joint_states.json, instruction.txt, labels.json
├── cache/                    # VLM response cache
├── out/runs/<timestamp>/     # every run: config snapshot, raw responses, parsed JSON, figures
├── logs/
└── .venv/
```

---

## 5. Phases

Timeline: Wyatt is at ITRI on Mon 9/14 and Tue 9/15, with limited review time. Phases 0–2 are the guaranteed demo; 3–4 are the target; 5 is a stretch.

| When | Phase | Result |
|---|---|---|
| Fri 9/11 – Sun 9/13 | 0 Recon, 1 Data capture | system map, 8–12 scenes on disk |
| Mon 9/14 – Tue 9/15 | 2 Grounding, 3 Lifting | offline 2D and 3D plans on real images |
| Wed 9/16 | 4 Evaluation, 5 (if approved) | results + demo assets; dry-run IK with Tuan |
| Thu 9/17 | Demo | — |

### Phase 0 — System reconnaissance (read-only; no writes outside `WORK_DIR`)

**Goal**: an accurate map of the current pipeline, so the new module plugs in without changing the existing system.

**Tasks**:
1. Locate the robot repos and the local-VLM code. Record their paths and `git rev-parse HEAD` (reading only).
2. Trace the grasp pipeline end to end, with `file:line` for each step:
   - image acquisition (topics, resolution, whether depth is aligned to color)
   - VLM prompt construction and response parsing (confirm the `[y, x]` order and normalization)
   - current model string
   - pixel → 3D deprojection (window/median? which intrinsics?)
   - camera → base transform (TF lookup, or FK + hand-eye matrix? where is the extrinsic stored?)
   - approach/grasp offsets
   - IK call signature (`piper_kinematic.py`)
   - how release is done
3. Read-only runtime inspection, only if the system is running: `ros2 node list`, `ros2 topic list`, `ros2 topic info`, `ros2 topic hz` for camera topics, `ros2 topic echo --once` for `camera_info` and `joint_states`, and `tf2_echo` between the camera optical frame and the base frame.
4. Environment check: ROS 2 distro, Python version, availability of `numpy`, `opencv`, `open3d`, `pyrealsense2`, `google-genai`, `rclpy`, and `roboticstoolbox`; existing Ollama models (`ollama list`); free disk space; whether the network can reach the Gemini API.
5. Find where the local-VLM replacement plugs into the pipeline and how its interface differs from `gemini_client.py`.

**Must-confirm checklist** (answer each in `docs/00_system_recon.md` as CONFIRMED with `file:line`, or UNVERIFIED):
- [ ] exact paths of the files listed in 2.1, plus the repo each belongs to
- [ ] RGB and depth topic names, resolution, aligned-depth availability, depth units (mm vs. m)
- [ ] intrinsics source
- [ ] name of the camera optical frame and the base frame; hand-eye extrinsic location
- [ ] the point convention used by the parser
- [ ] the current Gemini model string
- [ ] IK function inputs/outputs and joint-limit handling
- [ ] whether pick and release happen at different base locations in the current scenario library
- [ ] the local-VLM interface: model name, call function, measured latency if documented

**Deliverables**:
- `docs/00_system_recon.md`: pipeline diagram (text/mermaid), the checklist, and a list of integration points
- `docs/questions.md`
- the Phase 0 report

**STOP. Wait for approval.**

### Phase 1 — Scene capture (subscriber-only script)

**Goal**: a small real dataset from Robot 129's own camera, so all development runs offline.

**Tasks**:
1. Write `scripts/capture_scene.py`. It subscribes to color, aligned depth, camera_info, joint_states, and the TF camera→base, saves **one synchronized snapshot** into `data/scenes/<scene_id>/`, and writes `instruction.txt` from a CLI argument. No publishers anywhere.
2. Write a `labels.json` template and a minimal OpenCV click tool (`scripts/label_scene.py`). Wyatt uses it to draw bounding boxes for `target`, `destination`, and `obstacles[]`.
3. Scene spec (Wyatt arranges the scenes; the arm stays still, looking down at a tabletop). **The object and the destination must both be visible in the same image.**
   - S1 simple pick-and-place, no obstacle (3–4 scenes)
   - S2 an obstacle between object and destination, e.g. a tall bottle (3–4 scenes)
   - S3 clutter with distractor objects (2–4 scenes)
   - Instructions: direct ("put the cup in the bowl") and indirect ("put the red thing where it belongs")
4. Running the capture script requires Wyatt's approval the first time. Wyatt may prefer to run it himself.

**Deliverables**: the scripts, captured scenes (by Wyatt), a data README, and the Phase 1 report. **STOP.**

### Phase 2 — Multi-point 2D grounding (offline)

**Goal**: from one image and one instruction, produce a validated plan with grasp / waypoint / release 2D points.

**Tasks**:
1. **Schema** (`src/mpg/schema.py`, pydantic or dataclasses). Plan JSON v0:
   ```json
   {
     "scene_id": "s2_03",
     "instruction": "put the cup in the bowl",
     "backend": "gemini-robotics-er-2-preview",
     "stage_a": {
       "view_description": "...",
       "mode": "pick",
       "target": "white cup",
       "destination": "blue bowl",
       "tool": null
     },
     "steps": [
       {"step": 0, "type": "GRASP",   "desc": "cup body", "geometric_meaning": "...", "point_yx": [412, 388]},
       {"step": 1, "type": "WAYPOINT","desc": "above the bottle, toward the bowl", "geometric_meaning": "...", "point_yx": [300, 520]},
       {"step": 2, "type": "RELEASE", "desc": "center of bowl", "geometric_meaning": "...", "point_yx": [455, 690]}
     ],
     "meta": {"latency_s": 0.0, "n_calls": 2, "parse_retries": 0}
   }
   ```
   - Tool-use (`mode: "tool"`) is parsed and stored, but flagged `unsupported_by_executor`.
   - The executor only needs pick-and-place for this sprint.
2. **Prompts** (`prompts/`), adapted from ZeroDex App. D.2 following the rules in 3.1:
   - `stage_a.txt`: view description, roles, mode, and a text-only plan with `geometric_meaning` (no coordinates). This is the single-view version of "Plan Selector".
   - `stage_b.txt`: point localization for the fixed plan. This is the single-view version of "Point Localization from a Fixed Plan".
   - `single_call.txt`: variant that does A and B in one call (Tuan's "one VLM call" idea).
   - `baseline_copied.txt`: the current system's pointing prompt, copied verbatim with provenance. The baseline issues one call for the target point and one call for the destination point.
3. **Backends** (`src/mpg/vlm/`):
   - `gemini.py`: model from config; response caching; call logging.
   - `local.py`: reuse the interface from `LOCAL_VLM_CODE`.
   - Both expose the same `query(image, prompt) -> str`.
   - Parse robustly: strip code fences, validate against the schema, retry once on failure, and record every failure.
4. **Visualization** (`src/mpg/viz.py`): overlay colored points (grasp red, waypoint blue, release green, matching ZeroDex Fig. S1), labels, and a legend. Put the baseline and new outputs side by side.
5. Run on all scenes with Gemini ER 2 first. Run the local backend only after Gemini works.

**Definition of done**: every scene has a parsed plan JSON and an overlay image under `out/runs/<ts>/`; parse-failure rate and latency are reported; unit tests pass for parsing and for `[y, x]`→pixel conversion. **STOP.**

### Phase 3 — Single-view RGB-D lifting to 3D + collision-aware refinement (offline)

**Goal**: convert each 2D step into a 3D point in the robot base frame that an executor could use.

Use the same conventions as the existing system; reuse its intrinsics and transform code read-only if possible. Put all parameters in `configs/default.yaml`.

1. **Pixel conversion**: `u = x/1000 * W`, `v = y/1000 * H`. Unit-test this; it is the most common bug.
2. **Depth**: median of valid depth in a k×k window (default 7×7; ignore zeros). Report the valid fraction. Fail the point if the valid fraction is below the threshold.
3. **Deprojection** with the recorded intrinsics, then camera → base transform using the recorded TF (or FK + hand-eye, matching the existing code).
4. **Point-type rules** (single-view depth hits surfaces, so the waypoint and release need height rules):
   - **GRASP**: the deprojected surface point, plus the existing system's grasp offsets.
   - **Object height estimate**: from the point cloud around the grasp point, top of object minus table plane. Fit the table plane with RANSAC on the scene cloud.
   - **RELEASE**: XY from the deprojected release pixel; Z = destination surface + object height + `release_clearance`.
   - **WAYPOINT**: XY from the deprojected waypoint pixel; Z = max scene height inside a corridor (radius `corridor_r`) along the segment grasp→release, + object height + `waypoint_clearance`.
5. **Collision-aware refinement** (ZeroDex Eq. 12 / App. D.1, simplified):
   - Approximate the carried object as an axis-aligned box from its point-cloud extent.
   - For RELEASE and WAYPOINT, test overlap with scene points (excluding the object's own points).
   - If a collision is found, search a local grid of ±4 cm at 2 cm resolution, trying vertical offsets before lateral ones, and take the nearest collision-free point.
   - If none is found, mark the step `refine_failed`.
6. **Output**: plan JSON v0 plus `position_base_m`, `frame_id`, `depth_valid_frac`, and `refined: bool` per step. Also render a 3D point-cloud screenshot with the three spheres (Open3D offscreen), like ZeroDex Fig. S1.

**Definition of done**: 3D plans and screenshots for all scenes; a sanity check showing the table plane height is consistent across scenes (report the spread); a list of failed points with their reasons. **STOP.**

### Phase 4 — Evaluation and demo assets

**Metrics** (computed against Wyatt's `labels.json`; follow ZeroDex Sec 4.3 in spirit):

| Metric | Definition |
|---|---|
| Grasp-on-target rate | GRASP 2D point inside the target bbox |
| Release-in-destination rate | RELEASE 2D point inside the destination bbox |
| Waypoint validity | waypoint ≠ release (pixel distance > threshold), not inside any obstacle bbox, and (3D) collision-free after refinement |
| Parse success rate | valid schema on first try / after retry |
| Consistency | repeat each query 3× and report the pixel std per step type |
| 3D validity | depth-valid fraction; refinement success |
| Cost | VLM calls per plan; latency p50/p90 per backend |

**Comparisons** (one variable at a time):
- (a) baseline single-point vs. 2-call multi-point, both on Gemini ER 2
- (b) 2-call vs. 1-call multi-point on Gemini ER 2
- (c) Gemini ER 2 vs. local VLM with the best prompt from (b)

**Deliverables**:
- `docs/02_results.md`: tables, representative success and failure cases, failure taxonomy
- `out/demo/`: 4–6 best overlay images, 2–3 3D screenshots, one side-by-side baseline-vs-new figure
- `scripts/make_demo_assets.py`
- the Phase 4 report

**STOP.**

### Phase 5 — Integration dry-run with Tuan's executor (only with explicit approval)

1. Write `docs/interface_contract.md`: the 3D plan JSON is the handoff. The executor (Tuan) consumes the steps in order: GRASP → lift → WAYPOINT → RELEASE.
2. Write `scripts/dry_run_ik.py`: for each 3D point, call the existing IK function (imported read-only) and report the joint solution, joint-limit margin, and success/failure. **It computes only and sends nothing.**
3. Optional, with separate approval: a ROS 2 node in `WORK_DIR` that publishes the plan JSON as `std_msgs/String` on a **new** topic `/wyatt_mpg/plan` only. It must never publish on any existing topic.
4. Physical execution is done by Wyatt/Tuan under their own supervision, outside this task.

### Design-only document (no implementation this sprint)
Write `docs/future_active_multiview.md`, about one page:
- how to capture 2–3 wrist-camera views by moving the arm (poses chosen by humans)
- how to obtain camera extrinsics per view from FK + hand-eye
- how to apply ZeroDex RANSAC triangulation (ε = 20 px at 640×480, τ = 0.5·M) and reference-ray voting (depth 0.5–2.0 m, 0.05 m step)
- what could go wrong: wrist-camera baseline length, motion blur, calibration error

---

## 6. Coding standards
- Python 3, type hints, small modules. `ruff` for lint if available in the venv.
- Every run writes a config snapshot, raw VLM text, parsed JSON, and figures to `out/runs/<timestamp>/`, so results are reproducible.
- No hard-coded thresholds in code; they go in `configs/default.yaml`.
- Unit tests for parsing, coordinate conversion, deprojection (with a synthetic depth image), and the refinement grid search.
- Commit inside `WORK_DIR` at the end of each phase, with a clear message.

---

## 7. Phase report template (`docs/phase_reports/phase_N.md`)

```
# Phase N report — <date>
## Done (each item with output path)
## Verified (how: command run / file:line / test name)
## NOT RUN / UNVERIFIED
## Numbers (if any; with source file)
## Problems and discrepancies vs. AGENTS.md
## Questions for Wyatt (numbered, answerable in one line each)
## Proposed next step
```

End every phase by printing the report path and stopping.
