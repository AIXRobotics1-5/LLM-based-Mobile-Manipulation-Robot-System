"""One-time script: Convert E0509+Gripper URDF to USD for Isaac Sim.

Run once:
    source ~/isaacsim_env/bin/activate
    cd ~/doosan_ws/src/RB/RB/scripts
    python convert_urdf_to_usd.py

Generates: ../asset/e0509_gripper.usd
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": True})

import os

# Paths
URDF_PATH = os.path.expanduser(
    "~/doosan_ws/src/e0509_gripper_description/config/curobo/e0509_gripper.urdf"
)
OUTPUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "asset"))
OUTPUT_USD = os.path.join(OUTPUT_DIR, "e0509_gripper.usd")

# Package path mapping for mesh resolution
PACKAGE_PATHS = {
    "dsr_description2": os.path.expanduser(
        "~/doosan_ws/src/doosan-robot2/dsr_description2"
    ),
    "rh_p12_rn_a_description": os.path.expanduser(
        "~/doosan_ws/src/RH-P12-RN-A/rh_p12_rn_a_description"
    ),
}


def main():
    import omni.kit.commands
    import omni.usd

    print(f"[Convert] URDF: {URDF_PATH}")
    print(f"[Convert] Output: {OUTPUT_USD}")

    # Read and fix package:// paths in URDF
    with open(URDF_PATH, "r") as f:
        urdf_content = f.read()

    for pkg_name, pkg_path in PACKAGE_PATHS.items():
        urdf_content = urdf_content.replace(
            f"package://{pkg_name}/", f"{pkg_path}/"
        )

    # Write temp URDF with resolved paths
    temp_urdf = os.path.join(OUTPUT_DIR, "_temp_e0509_gripper.urdf")
    with open(temp_urdf, "w") as f:
        f.write(urdf_content)
    print(f"[Convert] Resolved package paths → {temp_urdf}")

    # Step 1: Create import config
    status, import_config = omni.kit.commands.execute(
        "URDFCreateImportConfig"
    )
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.create_physics_scene = False
    import_config.import_inertia_tensor = True
    import_config.default_drive_type = 1  # position drive
    import_config.default_drive_strength = 1e4
    import_config.default_position_drive_damping = 1e3

    # Step 2: Import URDF to current stage and save as USD
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=temp_urdf,
        import_config=import_config,
        dest_path=OUTPUT_USD,
    )
    print(f"[Convert] Import status: {status}, prim: {prim_path}")

    # Save the stage as USD
    stage = omni.usd.get_context().get_stage()
    if stage:
        stage.Export(OUTPUT_USD)
        print(f"[Convert] USD exported to: {OUTPUT_USD}")

    # Cleanup temp
    if os.path.exists(temp_urdf):
        os.remove(temp_urdf)
    print("[Convert] Done!")


main()
simulation_app.close()
