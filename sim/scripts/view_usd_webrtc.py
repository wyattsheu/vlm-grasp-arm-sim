"""Generic USD WebRTC viewer + inspector.

Loads ANY USD/USDA/USDC file, drops it into a standard inspection scene (deep
gray floor, dome light, 1m grid, 10cm reference cube, world axes), auto-frames
the camera on the model's bounding box, streams the result over WebRTC (same
signaling/stream ports as the Robot 129 viewer), prints a plain-text
inspection report (units, dimensions, prim tree, mesh/material/physics
sanity checks), and saves a screenshot.

This does NOT replace real validation -- it tells you whether the model looks
reasonable and is loadable/physically stable, not whether it is dimensionally
correct against a real object. Use `--physics` to also drop it under gravity
onto the floor for a few seconds and check it doesn't explode/sink through.

Usage (normally invoked via tools/start_usd_webrtc.sh, not directly):
    uv run --no-sync python view_usd_webrtc.py --usd /abs/path/model.usd \
        --livestream 2 --kit_args "..." --hold-seconds -1
"""

import argparse
import json
import time
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--usd", type=Path, required=True, help="Absolute path to the USD/USDA/USDC file to inspect")
parser.add_argument("--out-dir", type=Path, default=None, help="Where to write report.json/screenshot.png (default: alongside this bundle's out/webrtc_usd_viewer)")
parser.add_argument("--physics", action="store_true", help="Enable physics, drop the model onto the floor, and check for explosions/NaN")
parser.add_argument("--physics-seconds", type=float, default=3.0)
parser.add_argument("--warmup-frames", type=int, default=200)  # more RTX samples -> less grainy fine detail (bumps etc.)
parser.add_argument("--preroll-seconds", type=float, default=5.0)
parser.add_argument("--hold-seconds", type=float, default=-1.0, help="-1 = hold forever (stream until killed)")
parser.add_argument("--realtime", action="store_true", default=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = False
launcher = AppLauncher(args)
app = launcher.app

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, UsdLux, Gf, Sdf  # noqa: E402

try:
    # Only available when the viewport/livestream extensions are actually
    # loaded (i.e. --livestream 1/2). Degrades gracefully with --livestream 0
    # so this script also works as a fast "just check the report" offline
    # smoke test with no WebRTC/GPU-viewport cost.
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
except ModuleNotFoundError:
    get_active_viewport = lambda: None  # noqa: E731
    capture_viewport_to_file = None

TAG = "[USD WEBRTC]"

# Prims commonly present in Isaac-Sim-authored USD exports that are scene
# infrastructure, not "the model" -- excluded from the bounding-box/report so
# a 5m ground plane or physics scene doesn't blow out the auto-frame camera
# and dimension report. Matched case-insensitively against the prim name.
INFRA_PRIM_NAME_MARKERS = ("groundplane", "physicsscene", "floor")


def log(msg: str) -> None:
    print(f"{TAG} {msg}", flush=True)


def set_live_viewport_camera(viewport, eye, target) -> None:
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    camera_path = viewport.get_active_camera() or "/OmniverseKit_Persp"
    camera_state = ViewportCameraState(camera_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(*map(float, eye)), False)
    camera_state.set_target_world(Gf.Vec3d(*map(float, target)), True)
    log(f"CAMERA - eye={tuple(round(v, 3) for v in eye)} target={tuple(round(v, 3) for v in target)}")


def draw_reference_scene(stage, model_root: str, center, radius: float):
    """1m grid + world axes via debug_draw, plus a 10cm reference cube prim."""
    try:
        from isaacsim.util.debug_draw import _debug_draw
        draw = _debug_draw.acquire_debug_draw_interface()
    except Exception as exc:  # pragma: no cover - extension not enabled
        carb.log_warn(f"debug_draw unavailable ({exc}); skipping grid/axes overlay")
        draw = None

    if draw is not None:
        starts, ends, colors, widths = [], [], [], []
        n = max(6, int(radius / 1.0) + 4)
        grid_color = carb.Float4(0.4, 0.4, 0.42, 0.6)
        for i in range(-n, n + 1):
            starts.append(carb.Float3(float(i), float(-n), 0.0))
            ends.append(carb.Float3(float(i), float(n), 0.0))
            starts.append(carb.Float3(float(-n), float(i), 0.0))
            ends.append(carb.Float3(float(n), float(i), 0.0))
            colors.extend([grid_color, grid_color])
            widths.extend([1.0, 1.0])
        axis_len = max(0.3, radius * 0.6)
        axes = [
            (carb.Float3(0, 0, 0), carb.Float3(axis_len, 0, 0), carb.Float4(0.9, 0.15, 0.15, 1.0)),  # X red
            (carb.Float3(0, 0, 0), carb.Float3(0, axis_len, 0), carb.Float4(0.15, 0.9, 0.15, 1.0)),  # Y green
            (carb.Float3(0, 0, 0), carb.Float3(0, 0, axis_len), carb.Float4(0.15, 0.4, 0.95, 1.0)),  # Z blue
        ]
        for s, e, c in axes:
            starts.append(s); ends.append(e); colors.append(c); widths.append(3.0)
        draw.draw_lines(starts, ends, colors, widths)
        log(f"GRID - {2*n}x{2*n} m grid drawn, axes length={axis_len:.2f}m (X=red Y=green Z=blue)")

    # 10cm reference cube, offset to the side so it never occludes the model
    ref_path = "/World/Reference10cmCube"
    ref = UsdGeom.Cube.Define(stage, ref_path)
    ref.CreateSizeAttr(0.1)
    UsdGeom.XformCommonAPI(ref).SetTranslate(Gf.Vec3d(center[0] + radius + 0.3, center[1] - radius - 0.3, 0.05))
    ref.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.6, 0.1)])
    log(f"REFERENCE_CUBE - 10cm cube placed at {ref_path} next to the model for scale comparison")


def _is_infra_name(name: str) -> bool:
    lname = name.lower()
    return any(marker in lname for marker in INFRA_PRIM_NAME_MARKERS)


def inspect_model(stage, model_root_path: str) -> dict:
    """Plain-text-friendly report: units, dims, prim tree, mesh/material/physics sanity."""
    root_prim = stage.GetPrimAtPath(model_root_path)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)

    # Union per-prim bounds, SKIPPING infra prims (ground planes, physics
    # scenes) -- a naive ComputeWorldBound(root) picks those up too and can
    # make e.g. a 5m ground plane dominate a 10cm model's reported size and
    # auto-frame camera. See INFRA_PRIM_NAME_MARKERS.
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    combined_range = Gf.Range3d()
    skipped_infra = []

    prim_paths, meshes, materials_bound, materials_missing = [], [], 0, 0
    n_faces = n_verts = n_degenerate = 0
    has_rigid_body = has_collision = has_mass = has_joints = False

    # NOTE: PruneChildren() lives on the Python *iterator* object in this
    # pxr build (iter(Usd.PrimRange(...))), not on the PrimRange itself --
    # that differs between Isaac Sim installs, so don't assume one or the
    # other works without checking (verified via .pyi inspection here).
    prim_iter = iter(Usd.PrimRange(root_prim))
    for prim in prim_iter:
        if prim != root_prim and _is_infra_name(prim.GetName()):
            skipped_infra.append(str(prim.GetPath()))
            prim_iter.PruneChildren()
            continue
        prim_paths.append(str(prim.GetPath()))
        if prim.IsA(UsdGeom.PointInstancer):
            # BBoxCache on the instancer itself correctly covers all its
            # instances. But do NOT also descend into its "prototypes" scope
            # below -- that visits the raw prototype prims at their OWN
            # (usually origin) transform, which would double-count/pollute
            # the bbox with e.g. a radius-1.0 placeholder sphere sitting at
            # (0,0,0) (this exact bug briefly reported a ~18cm object as a
            # 2x2x2m box). Tried fixing this by having the asset mark its
            # prototype's purpose as "guide" instead -- don't do that either,
            # it hid the rendered instances too (purpose is commonly
            # inherited by instances, not just the prototype prim). The bbox
            # special-case here is the correct fix, kept local to this tool.
            b = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not b.IsEmpty():
                combined_range = Gf.Range3d.GetUnion(combined_range, b)
            prim_iter.PruneChildren()
            continue
        if prim.IsA(UsdGeom.Boundable):
            b = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not b.IsEmpty():
                combined_range = Gf.Range3d.GetUnion(combined_range, b)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            has_rigid_body = True
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            has_collision = True
        if prim.HasAPI(UsdPhysics.MassAPI):
            has_mass = True
        if prim.IsA(UsdPhysics.Joint) if hasattr(UsdPhysics, "Joint") else False:
            has_joints = True
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            pts = mesh.GetPointsAttr().Get()
            fvc = mesh.GetFaceVertexCountsAttr().Get()
            if pts is not None:
                n_verts += len(pts)
            if fvc is not None:
                n_faces += len(fvc)
                n_degenerate += sum(1 for c in fvc if c < 3)
            binding = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if binding and binding.GetPrim().IsValid():
                materials_bound += 1
            else:
                materials_missing += 1
            meshes.append(str(prim.GetPath()))

    if not combined_range.IsEmpty():
        dims = [combined_range.GetMax()[i] - combined_range.GetMin()[i] for i in range(3)]
        center = [(combined_range.GetMax()[i] + combined_range.GetMin()[i]) / 2 for i in range(3)]
    else:
        dims = [0.0, 0.0, 0.0]
        center = [0.0, 0.0, 0.0]

    report = {
        "usd_file": str(args.usd),
        "stage_meters_per_unit": meters_per_unit,
        "stage_up_axis": str(up_axis),
        "model_root": model_root_path,
        "bbox_dims_m": [round(d * meters_per_unit if meters_per_unit != 1.0 else d, 4) for d in dims],
        "bbox_center_m": [round(c, 4) for c in center],
        "excluded_infra_prims": skipped_infra,
        "n_prims": len(prim_paths),
        "n_meshes": len(meshes),
        "n_faces_total": n_faces,
        "n_vertices_total": n_verts,
        "n_degenerate_faces": n_degenerate,
        "meshes_with_material": materials_bound,
        "meshes_without_material": materials_missing,
        "has_rigid_body_api": has_rigid_body,
        "has_collision_api": has_collision,
        "has_mass_api": has_mass,
        "has_joints": has_joints,
        "mesh_prim_paths": meshes[:50],
        "top_level_children": [str(c.GetPath()) for c in root_prim.GetChildren()],
    }
    return report


def print_report(report: dict) -> None:
    log("---- INSPECTION REPORT ----")
    log(f"file            = {report['usd_file']}")
    log(f"units           = {report['stage_meters_per_unit']} meters/unit (1.0 = correctly meters)")
    log(f"up axis         = {report['stage_up_axis']}")
    d = report["bbox_dims_m"]
    log(f"bbox size (m)   = {d[0]:.4f} x {d[1]:.4f} x {d[2]:.4f}  (x, y, z)")
    log(f"bbox center (m) = {report['bbox_center_m']}")
    if report["excluded_infra_prims"]:
        log(f"(excluded from bbox as scene infra: {report['excluded_infra_prims']})")
    log(f"prims / meshes  = {report['n_prims']} prims, {report['n_meshes']} meshes")
    log(f"faces / verts   = {report['n_faces_total']} faces, {report['n_vertices_total']} vertices")
    if report["n_degenerate_faces"] > 0:
        log(f"WARNING: {report['n_degenerate_faces']} degenerate (< 3 vertex) faces found")
    log(f"materials       = {report['meshes_with_material']} meshes bound, {report['meshes_without_material']} missing material")
    if report["meshes_without_material"] > 0:
        log("WARNING: some meshes have no material bound -- will render as default gray/pink")
    log(f"physics APIs    = rigid_body={report['has_rigid_body_api']} collision={report['has_collision_api']} mass={report['has_mass_api']} joints={report['has_joints']}")
    log(f"top-level prims = {report['top_level_children']}")
    log("---- END REPORT ----")


def main() -> int:
    out_dir = args.out_dir or (Path(__file__).resolve().parents[2] / "out" / "webrtc_usd_viewer")
    out_dir.mkdir(parents=True, exist_ok=True)

    usd_path = args.usd.resolve()
    if not usd_path.is_file():
        log(f"FATAL - USD file not found: {usd_path}")
        return 1

    settings = carb.settings.get_settings()
    settings.set("/rtx/background/source/type", 2)
    settings.set("/rtx/background/source/color", (0.055, 0.065, 0.080))

    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    stage = usd_context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # -- floor + light -----------------------------------------------------
    floor = UsdGeom.Cube.Define(stage, "/World/DeepGrayFloor")
    floor.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(floor).SetScale(Gf.Vec3f(4.0, 4.0, 0.025))
    UsdGeom.XformCommonAPI(floor).SetTranslate(Gf.Vec3d(0, 0, -0.0125))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.11, 0.12, 0.14)])
    if args.physics:
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    light = stage.DefinePrim("/World/Light", "DomeLight")
    light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1400.0)
    light.CreateAttribute("inputs:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.82, 0.84, 0.88))

    # A flat dome-only light leaves fine surface detail (bumps, dents,
    # normal-map-scale features) nearly invisible -- everything gets roughly
    # the same flat illumination from every direction. Add a angled key light
    # so real shading contrast/shadows show that detail. Dome intensity is
    # halved above to keep it from washing this back out.
    key_light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    key_light.CreateIntensityAttr(3000.0)
    key_light.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    key_light.CreateAngleAttr(2.0)  # soft-ish shadows, not a razor-sharp sun
    UsdGeom.XformCommonAPI(key_light).SetRotate(Gf.Vec3f(-45.0, 35.0, 0.0))

    # -- load the model as a reference under /World/Model ------------------
    model_path = "/World/Model"
    model_prim = stage.DefinePrim(model_path, "Xform")
    model_prim.GetReferences().AddReference(str(usd_path))
    log(f"LOADED - referenced {usd_path} at {model_path}")

    report = inspect_model(stage, model_path)
    print_report(report)
    (out_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")

    dims = report["bbox_dims_m"]
    center = report["bbox_center_m"]
    # BUG FIX (2026-09-16): this used to floor at 0.3m, so anything smaller
    # than a 30cm object (e.g. a ~18cm mug) got framed at the same distance
    # as a 30cm object -- "too far away to see detail" per user report. Floor
    # is now tiny (2cm) purely to avoid a zero-size degenerate case; real
    # objects are framed based on their own actual size.
    radius = max(0.02, (max(dims) if any(dims) else 1.0))
    UsdGeom.XformCommonAPI(model_prim).SetTranslate(Gf.Vec3d(0, 0, -center[2] + dims[2] / 2 if dims[2] else 0))
    # recompute center after re-grounding the model on the floor
    center = [center[0], center[1], dims[2] / 2 if dims[2] else center[2]]
    draw_reference_scene(stage, model_path, center, radius)

    # BUG FIX (2026-09-16, round 2): the viewer used to only step physics for
    # a fixed --physics-seconds drop test, then hold the LAST frame forever
    # -- i.e. by the time you connect, everything is a frozen snapshot. You
    # can't grab, push, or pull a frozen snapshot. Physics now runs
    # continuously through the whole hold loop below (whether or not
    # --physics was passed) so the scene is actually live while you're
    # connected -- WASD/mouse plus a live PhysX scene is what makes an
    # object interactable at all.
    physx_iface = physx_sim_iface = None
    if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid() and not any(
        "physicsscene" in p.lower() for p in report["top_level_children"]
    ):
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    if report["has_rigid_body_api"]:
        from omni.physx import get_physx_interface, get_physx_simulation_interface
        physx_iface = get_physx_interface()
        physx_sim_iface = get_physx_simulation_interface()
        physx_iface.start_simulation()
        log("PHYSICS - live: the scene keeps simulating for as long as you're connected")
    else:
        log("PHYSICS - model has no RigidBodyAPI; nothing to simulate/interact with")

    # BUG FIX: WASD move speed is a fixed absolute (m/s) Kit setting, so on a
    # ~15cm object framed up close, one tap covered many object-widths --
    # "jumps to somewhere far away" per user report. Scale it to the actual
    # object size instead of leaving Kit's larger-scene-oriented default.
    carb.settings.get_settings().set("/persistent/app/viewport/camMoveVelocity", max(0.02, radius * 0.5))

    dt = 1.0 / 60.0
    t = 0.0

    def step():
        """One render tick, plus one physics tick if physics is live -- use
        this everywhere below instead of a bare app.update() so the object
        keeps moving/simulating instead of sitting frozen the moment you
        connect."""
        nonlocal t
        if physx_sim_iface is not None:
            physx_sim_iface.simulate(dt, t)
            physx_sim_iface.fetch_results()
            t += dt
        app.update()

    step()
    step()
    viewport = get_active_viewport()
    eye = (center[0] + radius * 1.8, center[1] - radius * 1.8, center[2] + radius * 1.4)
    if viewport is not None:
        set_live_viewport_camera(viewport, eye=eye, target=tuple(center))

    log(f"WARMING_UP - {args.warmup_frames} rendered frames")
    for _ in range(max(0, args.warmup_frames)):
        step()

    if viewport is not None:
        set_live_viewport_camera(viewport, eye=eye, target=tuple(center))
    for _ in range(15):
        step()

    screenshot_path = out_dir / "screenshot.png"
    if viewport is not None and capture_viewport_to_file is not None:
        capture_viewport_to_file(viewport, file_path=str(screenshot_path))
        for _ in range(60):
            step()
            if screenshot_path.is_file() and screenshot_path.stat().st_size > 1024:
                break
        log(f"SCREENSHOT - saved {screenshot_path}")
    else:
        log("SCREENSHOT - skipped (no viewport; run with --livestream 1 or 2 for a screenshot)")

    if args.physics and physx_sim_iface is not None:
        n_steps = int(round(args.physics_seconds / dt))
        for _ in range(n_steps):
            step()
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        rng2 = bbox_cache.ComputeWorldBound(model_prim).ComputeAlignedRange()
        z_min_after = rng2.GetMin()[2] if not rng2.IsEmpty() else float("nan")
        exploded = not np.isfinite(z_min_after) or abs(z_min_after) > 50.0
        log(f"PHYSICS_RESULT - after {args.physics_seconds:.1f}s: lowest point z={z_min_after:.4f}m "
            f"(negative = sinking through floor; {'EXPLODED/NaN' if exploded else 'stable'})")

    log("READY - streaming; connect over WebRTC now. Left-click-drag directly on "
        "the object to push/pull it (standard Omniverse physics-interaction "
        "convention) -- physics keeps running the whole time you're connected.")
    remaining = None if args.hold_seconds < 0 else max(0, round(args.hold_seconds * 30))
    while app.is_running() and (remaining is None or remaining > 0):
        step()
        if remaining is not None:
            remaining -= 1
        time.sleep(1 / 30)

    log("COMPLETE - shutting down")
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
    except Exception:
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        app.close()
    raise SystemExit(exit_code)
