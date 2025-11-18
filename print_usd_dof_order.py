
# print_usd_dof_order.py
# Usage:
#   ./isaaclab.sh -p scripts/tools/print_usd_dof_order.py --usd /path/to/g1.usd --prim /World/g1 --yaml_out g1.yaml

import argparse
import json
from pathlib import Path

# 1) Launch Kit headless
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import carb
import omni.usd
from pxr import Usd

def open_stage(usd_path: str):
    ctx = omni.usd.get_context()
    ctx.open_stage(usd_path)
    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError(f"Failed to open stage: {usd_path}")
    return stage

def prim_exists(stage: Usd.Stage, prim_path: str) -> bool:
    return stage.GetPrimAtPath(prim_path).IsValid()

def get_default_prim_path(stage: Usd.Stage) -> str:
    dp = stage.GetDefaultPrim()
    return dp.GetPath().pathString if dp and dp.IsValid() else "/"

def print_and_optionally_save(names, yaml_out=None):
    print("\n=== USD DOF / joint order (policy input order) ===")
    for i, n in enumerate(names):
        print(f"{i:02d}: {n}")
    print(f"Total DOFs: {len(names)}")

    if yaml_out:
        # write a very simple yaml mapping
        lines = ["usd_dof_order:"]
        for i, n in enumerate(names):
            lines.append(f"  {i}: {n}")
        Path(yaml_out).write_text("\n".join(lines), encoding="utf-8")
        print(f"\nSaved mapping to: {yaml_out}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", required=True, help="Path to the USD file (e.g., /path/to/g1.usd)")
    parser.add_argument("--prim", default=None, help="Articulation root prim path (e.g., /World/g1)")
    parser.add_argument("--yaml_out", default=None, help="Optional path to save a yaml mapping")
    args = parser.parse_args()

    stage = open_stage(args.usd)

    # If prim not given, try defaultPrim as a sensible guess
    prim_path = args.prim or get_default_prim_path(stage)
    if not prim_exists(stage, prim_path):
        raise RuntimeError(f"Prim not found: {prim_path}")

    # 2) Try Isaac Sim Core Articulation first (works across most Isaac versions)
    dof_names = None
    try:
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation

        world = World(physics_dt=1/120.0, rendering_dt=1/60.0, stage_units_in_meters=1.0)
        robot = Articulation(prim_path=prim_path, name="robot")
        world.scene.add(robot)
        world.reset()  # builds the articulation
        dof_names = list(robot.dof_names)
    except Exception as e:
        carb.log_warn(f"Core Articulation path failed: {e}")

    # 3) Fallback: Isaac Lab ArticulationView if available
    if dof_names is None:
        try:
            from omni.isaac.core import World  # still uses Core world
            from omni.isaac.core.articulations import ArticulationView

            world = World(physics_dt=1/120.0, rendering_dt=1/60.0)
            world.reset()
            view = ArticulationView(prim_paths_expr=prim_path)
            world.scene.add(view)
            world.reset()
            dof_names = list(view.get_dof_names())
        except Exception as e:
            carb.log_error(f"ArticulationView fallback also failed: {e}")
            raise

    print_and_optionally_save(dof_names, args.yaml_out)

if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
