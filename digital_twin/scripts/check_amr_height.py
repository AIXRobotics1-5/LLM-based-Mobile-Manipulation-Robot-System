"""Check IW Hub origin vs visual position."""
from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": True})

from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom, Gf
import omni.usd

create_new_stage()
stage = omni.usd.get_context().get_stage()

assets_root = get_assets_root_path()
add_reference_to_stage(usd_path=assets_root + "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd", prim_path="/AMR")

amr_prim = stage.GetPrimAtPath("/AMR")
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

print(f"\n{'='*60}")
# Check each major part with full bbox
for child in amr_prim.GetChildren():
    name = child.GetName()
    if "joint" in name or "material" in name:
        continue
    child_bbox = bbox_cache.ComputeWorldBound(child)
    child_range = child_bbox.ComputeAlignedRange()
    min_pt = child_range.GetMin()
    max_pt = child_range.GetMax()
    print(f"  {name:20s}  Z: {min_pt[2]:.4f} ~ {max_pt[2]:.4f}  "
          f"X: {min_pt[0]:.4f} ~ {max_pt[0]:.4f}  "
          f"Y: {min_pt[1]:.4f} ~ {max_pt[1]:.4f}")

# Check xformOp on root
print(f"\nRoot xformOps:")
xformable = UsdGeom.Xformable(amr_prim)
for op in xformable.GetOrderedXformOps():
    print(f"  {op.GetName()}: {op.Get()}")

# Check chassis xformOps
chassis = stage.GetPrimAtPath("/AMR/chassis")
if chassis:
    xf = UsdGeom.Xformable(chassis)
    print(f"\nChassis xformOps:")
    for op in xf.GetOrderedXformOps():
        print(f"  {op.GetName()}: {op.Get()}")

# Check lift
lift = stage.GetPrimAtPath("/AMR/lift")
if lift:
    xf = UsdGeom.Xformable(lift)
    print(f"\nLift xformOps:")
    for op in xf.GetOrderedXformOps():
        print(f"  {op.GetName()}: {op.Get()}")

# Where should the robot arm base be to sit ON TOP of the chassis?
overall_bbox = bbox_cache.ComputeWorldBound(amr_prim)
overall_range = overall_bbox.ComputeAlignedRange()
print(f"\nOverall AMR:")
print(f"  Bottom Z (wheels): {overall_range.GetMin()[2]:.4f}")
print(f"  Top Z (highest):   {overall_range.GetMax()[2]:.4f}")
print(f"  Center X:          {(overall_range.GetMin()[0]+overall_range.GetMax()[0])/2:.4f}")
print(f"\n  → Robot arm base Z should be: {overall_range.GetMax()[2]:.4f}")
print(f"{'='*60}")

simulation_app.close()
