# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

import omni.ext
import omni.ui as ui
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics


ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/DigitalTwin/Assets/Warehouse"
)
RACK_ASSET = f"{ASSET_ROOT}/Shelving/Racks/Rack_A/Rack_A01_PR_NVD_01.usd"
LOADED_PALLET_ASSET = (
    f"{ASSET_ROOT}/Shipping/Cardboard_Boxes_on_Pallet/Pallet_Asm_A/"
    "Pallet_Asm_A04_120x122x75cm_PR_V_NVD_01.usd"
)
WOOD_PALLET_ASSET = (
    f"{ASSET_ROOT}/Shipping/Pallets/Wood/Block_A/BlockPallet_A01_PR_NVD_01.usd"
)
BIN_ASSET = f"{ASSET_ROOT}/Storage/Bins/Bin_A/Bin_A06_30x51x19cm_PR_V_NVD_01.usd"
GUARD_RAIL_ASSET = f"{ASSET_ROOT}/Safety/Barriers/GuardRail_A/GuardRail_A01_01.usd"

ISAAC_ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/4.5/Isaac/Robots"
)
# ForkliftC: driveable Isaac Sim robot asset (RigidBody + Articulation +
# Collision already authored on the prim, 7-joint steer/lift/mast rig).
FORKLIFT_ASSET = f"{ISAAC_ASSET_ROOT}/Forklift/forklift_c.usd"

# NVIDIA's warehouse assets above are authored in centimeters.  The digital
# twin stage uses meters, so raw USD references need this conversion at their
# reference root.  Asset-browser placement normally handles this through the
# Metrics Assembler, but AddReference() does not.
NVIDIA_ASSET_UNIT_SCALE = 0.01

# Isaac Sim robot assets (e.g. ForkliftC) are authored directly in meters, so
# no reference-time conversion is needed for them.
ISAAC_ROBOT_UNIT_SCALE = 1.0

# Measured source bounds converted to meters.  These are used for simple proxy
# colliders; visual assets keep their exact authored geometry.
RACK_COLLIDER_SIZE = (3.30, 0.60, 2.00)


def _set_transform(prim, position, rotation=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    """Author a translate/rotate/scale transform on a prim."""
    transform = UsdGeom.XformCommonAPI(prim)
    transform.SetTranslate(Gf.Vec3d(*position))
    transform.SetRotate(Gf.Vec3f(*rotation))
    transform.SetScale(Gf.Vec3f(*scale))


def _create_static_box(stage, path, position, scale, color, visible=True):
    """Create a stationary box collider, optionally visible."""
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    _set_transform(cube.GetPrim(), position, scale=scale)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    if not visible:
        cube.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    return cube.GetPrim()


def _create_dynamic_box(stage, path, position, scale, color, mass):
    """Create a dynamic test object affected by gravity and collisions."""
    prim = _create_static_box(stage, path, position, scale, color)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(mass)
    return prim


def _create_asset_reference(
    stage,
    path,
    asset_url,
    position,
    rotation=(0.0, 0.0, 0.0),
    scale=(1.0, 1.0, 1.0),
    instanceable=False,
    unit_scale=NVIDIA_ASSET_UNIT_SCALE,
):
    """Reference an NVIDIA asset into the meter stage, converting units."""
    xform = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    prim = xform.GetPrim()
    prim.GetReferences().ClearReferences()
    prim.GetReferences().AddReference(asset_url)
    prim.SetInstanceable(instanceable)
    meter_scale = tuple(component * unit_scale for component in scale)
    _set_transform(prim, position, rotation, meter_scale)
    return prim


class MyExtension(omni.ext.IExt):
    """Main extension for the forklift digital twin."""

    def on_startup(self, _ext_id):
        print("[my_company.forklift_twin.core] Extension startup")

        self._window = ui.Window("Forklift Digital Twin", width=340, height=280)
        with self._window.frame:
            with ui.VStack(spacing=8):
                ui.Label("Factory Setup", height=30)
                self._status_label = ui.Label(
                    "Ready. Online access is needed for NVIDIA assets.",
                    word_wrap=True,
                    height=45,
                )
                ui.Button("Create Factory", height=40, clicked_fn=self._create_factory)
                ui.Button("Add Forklift", height=32, clicked_fn=self._add_forklift)
                ui.Button("Open NVIDIA Assets", height=32, clicked_fn=self._open_asset_browser)
                ui.Button("Reset Factory", height=32, clicked_fn=self._reset_factory)
                ui.Label(
                    "Create the scene, select /World/Environment, press F, then press Play.",
                    word_wrap=True,
                )

    def _create_factory(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._status_label.text = "No USD stage is open."
            return

        self._status_label.text = "Creating a meter-scale factory and loading NVIDIA assets..."

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world.GetPrim())
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/Environment"))
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/Environment/Visuals"))
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/Environment/Colliders"))
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/Props"))
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/Vehicles"))

        physics_scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

        self._create_lighting(stage)
        self._create_shell(stage)
        self._create_warehouse_assets(stage)

        _create_dynamic_box(
            stage,
            "/World/Props/PhysicsTestBox",
            position=(0.0, 0.0, 3.0),
            scale=(0.5, 0.5, 0.5),
            color=(0.95, 0.45, 0.10),
            mass=5.0,
        )

        self._status_label.text = (
            "Factory created. Assets may take a moment to stream. Press Play to test physics."
        )

    @staticmethod
    def _create_lighting(stage):
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/DomeLight"))
        dome.CreateIntensityAttr().Set(650.0)

        sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/Sun"))
        sun.CreateIntensityAttr().Set(2500.0)
        sun.CreateAngleAttr().Set(1.0)
        _set_transform(sun.GetPrim(), (0.0, 0.0, 8.0), rotation=(35.0, -25.0, -35.0))

    @staticmethod
    def _create_shell(stage):
        _create_static_box(
            stage,
            "/World/Environment/Floor",
            position=(0.0, 0.0, -0.1),
            scale=(24.0, 16.0, 0.2),
            color=(0.24, 0.26, 0.29),
        )

        wall_specs = (
            ("NorthWall", (0.0, 8.1, 1.5), (24.0, 0.2, 3.0)),
            ("SouthWall", (0.0, -8.1, 1.5), (24.0, 0.2, 3.0)),
            ("EastWall", (12.1, 0.0, 1.5), (0.2, 16.0, 3.0)),
            ("WestWall", (-12.1, 0.0, 1.5), (0.2, 16.0, 3.0)),
        )
        for name, position, scale in wall_specs:
            _create_static_box(
                stage,
                f"/World/Environment/{name}",
                position=position,
                scale=scale,
                color=(0.50, 0.52, 0.55),
            )

    @staticmethod
    def _create_warehouse_assets(stage):
        # Two rack banks leave a broad central aisle for the future forklift.
        rack_specs = (
            ("Rack_01", (-7.0, 3.25, 0.0), 0.0),
            ("Rack_02", (-7.0, -1.25, 0.0), 0.0),
            ("Rack_03", (7.0, 3.25, 0.0), 180.0),
            ("Rack_04", (7.0, -1.25, 0.0), 180.0),
        )
        for name, position, yaw in rack_specs:
            _create_asset_reference(
                stage,
                f"/World/Environment/Visuals/Racks/{name}",
                RACK_ASSET,
                position,
                rotation=(0.0, 0.0, yaw),
                instanceable=True,
            )
            _create_static_box(
                stage,
                f"/World/Environment/Colliders/{name}",
                position=(position[0], position[1], RACK_COLLIDER_SIZE[2] / 2.0),
                scale=RACK_COLLIDER_SIZE,
                color=(1.0, 0.0, 1.0),
                visible=False,
            )

        _create_asset_reference(
            stage,
            "/World/Environment/Visuals/Pallets/LoadedPallet_01",
            LOADED_PALLET_ASSET,
            position=(-3.5, 4.5, 0.0),
        )
        _create_asset_reference(
            stage,
            "/World/Environment/Visuals/Pallets/WoodPallet_01",
            WOOD_PALLET_ASSET,
            position=(-3.5, 2.5, 0.0),
        )

        # The bins are kept at their real 51 x 30 x 19 cm dimensions.
        for index, y_position in enumerate((4.5, 3.75, 3.0), start=1):
            _create_asset_reference(
                stage,
                f"/World/Environment/Visuals/Bins/Bin_{index:02d}",
                BIN_ASSET,
                position=(3.5, y_position, 0.0),
                instanceable=True,
            )

        for index, x_position in enumerate((-2.5, 0.0, 2.5), start=1):
            _create_asset_reference(
                stage,
                f"/World/Environment/Visuals/Safety/GuardRail_{index:02d}",
                GUARD_RAIL_ASSET,
                position=(x_position, -5.5, 0.0),
                instanceable=True,
            )

    @staticmethod
    def _create_forklift(stage):
        # Spawn point: centered in the aisle, between the south wall and the
        # guard-rail line so it has room to maneuver in either direction.
        # Isaac robot assets are meter-authored, so unit_scale=1.0 (no cm
        # conversion). Facing (rotation yaw) is a guess -- check it visually
        # after Create Factory and adjust FORKLIFT_ROTATION below if the
        # forklift isn't pointing down the aisle.
        FORKLIFT_POSITION = (0.0, -6.75, 0.0)
        FORKLIFT_ROTATION = (0.0, 0.0, 0.0)
        _create_asset_reference(
            stage,
            "/World/Vehicles/Forklift",
            FORKLIFT_ASSET,
            position=FORKLIFT_POSITION,
            rotation=FORKLIFT_ROTATION,
            unit_scale=ISAAC_ROBOT_UNIT_SCALE,
        )

    def _add_forklift(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._status_label.text = "No USD stage is open."
            return

        if not stage.GetPrimAtPath("/World"):
            self._status_label.text = "Run Create Factory first so /World exists."
            return

        UsdGeom.Xform.Define(stage, Sdf.Path("/World/Vehicles"))
        self._create_forklift(stage)
        self._status_label.text = (
            "Forklift placed at /World/Vehicles/Forklift. Select it, press F to "
            "frame, and check it's facing down the aisle."
        )

    def _open_asset_browser(self):
        ui.Workspace.show_window("NVIDIA Assets")
        self._status_label.text = "NVIDIA Assets browser opened."

    def _reset_factory(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._status_label.text = "No USD stage is open."
            return

        for path in ("/World/Environment", "/World/Props", "/World/Lights", "/World/Vehicles"):
            if stage.GetPrimAtPath(path):
                stage.RemovePrim(path)

        self._status_label.text = "Factory content removed."

    def on_shutdown(self):
        print("[my_company.forklift_twin.core] Extension shutdown")
        self._window = None
