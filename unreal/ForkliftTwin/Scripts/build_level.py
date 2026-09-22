"""Create /Game/Maps/Warehouse: lighting, the warehouse, and the forklift at the Kit spawn point.

Run inside UE via -ExecutePythonScript after import_assets.py. The forklift is placed in the level
(so it is visible in the editor) and auto-possesses Player 0; ForkliftGameMode spawns one only if a
map has none.
"""
import unreal

MAP = "/Game/Maps/Warehouse"

level_lib = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def spawn(cls, location=(0, 0, 0), rotation=(0, 0, 0), label=None):
    """`rotation` is (pitch, yaw, roll) in degrees. Note unreal.Rotator's own positional order is
    (roll, pitch, yaw), so build it with keywords rather than unpacking a tuple into it."""
    pitch, yaw, roll = rotation
    actor = actors.spawn_actor_from_class(cls, unreal.Vector(*location), unreal.Rotator(roll=roll, pitch=pitch, yaw=yaw))
    if label:
        actor.set_actor_label(label)
    return actor


def main():
    if unreal.EditorAssetLibrary.does_asset_exist(MAP):
        unreal.EditorAssetLibrary.delete_asset(MAP)
    if not level_lib.new_level(MAP):
        raise RuntimeError("could not create level " + MAP)

    # Kit: DomeLight + DistantLight (35, -25, -35). UE equivalents: sky light + directional sun.
    # Lights must be Movable: spawned from script they default to Static, which needs baked lightmaps
    # and otherwise contributes no light at all (the level renders black).
    sun = spawn(unreal.DirectionalLight, (0, 0, 800), (-50, 35, 0), "Sun")
    sun_comp = sun.get_component_by_class(unreal.DirectionalLightComponent)
    sun_comp.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    sun_comp.set_editor_property("intensity", 10.0)
    spawn(unreal.SkyAtmosphere, label="SkyAtmosphere")
    sky = spawn(unreal.SkyLight, (0, 0, 400), label="SkyLight")
    sky_comp = sky.get_component_by_class(unreal.SkyLightComponent)
    sky_comp.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    sky_comp.set_editor_property("real_time_capture", True)
    sky_comp.set_editor_property("intensity", 1.5)  # a bit of extra fill for the shadowed sides of the open-topped hall
    spawn(unreal.ExponentialHeightFog, label="Fog")

    warehouse_cls = unreal.load_class(None, "/Script/ForkliftTwin.Warehouse")
    forklift_cls = unreal.load_class(None, "/Script/ForkliftTwin.ForkliftVehicle")
    spawn(warehouse_cls, label="Warehouse")

    # Kit spawn: (0, -6.75, 0) m, facing +X. UE: Y negated, cm; lifted 5 cm so the wheels settle onto the floor.
    spawn(forklift_cls, (0, 675, 5), (0, 0, 0), "Forklift")

    if not level_lib.save_current_level():
        raise RuntimeError("could not save " + MAP)
    unreal.log(f"[build_level] saved {MAP}")


main()
