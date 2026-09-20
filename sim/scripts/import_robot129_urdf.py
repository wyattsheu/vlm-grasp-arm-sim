"""Import the generated Robot 129 URDF into a versioned USD asset."""

import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--bundle", type=Path, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
launcher = AppLauncher(args)
app = launcher.app

import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "isaacsim.asset.importer.urdf", True
)

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from pxr import Usd, UsdPhysics


def main() -> int:
    bundle = args.bundle.resolve()
    pkg = bundle / "ros2_ws/src/robot129_description"
    out = bundle / "sim/assets/robot129"
    out.mkdir(parents=True, exist_ok=True)
    cfg = URDFImporterConfig(
        urdf_path=str(pkg / "urdf/robot129.urdf"),
        usd_path=str(out),
        merge_fixed_joints=False,
        merge_mesh=False,
        collision_from_visuals=False,
        allow_self_collision=False,
        ros_package_paths=[{"robot129_description": str(pkg)}],
        fix_base=True,
        joint_drive_type="force",
        joint_target_type="position",
        override_joint_stiffness={"joint[1-6]": 800.0, "joint[78]": 2000.0},
        override_joint_damping={"joint[1-6]": 80.0, "joint[78]": 100.0},
        run_asset_transformer=False,
    )
    usd_path = Path(URDFImporter(cfg).import_urdf())
    stage = Usd.Stage.Open(str(usd_path))
    prims = [str(p.GetPath()) for p in stage.Traverse()]
    revolute = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    prismatic = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.PrismaticJoint)]
    report = {
        "status": "PASS" if usd_path.is_file() and len(revolute) == 6 and len(prismatic) == 2 else "FAIL",
        "usd_path": str(usd_path), "prim_count": len(prims),
        "revolute_joints": revolute, "prismatic_joints": prismatic,
        "settings": cfg.__dict__,
    }
    (out / "import_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    try:
        status = main()
    finally:
        app.close()
    raise SystemExit(status)
