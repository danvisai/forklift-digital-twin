"""Import the baked forklift + warehouse USDs into /Game (run inside UE via -ExecutePythonScript).

Run bake_usd.py first (Scripts/setup.py does the whole chain). This creates:

    /Game/Warehouse/Materials/M_FlatColor, M_FlatGlass and one MI_<slot> per material slot
    /Game/Warehouse/Meshes/SM_<Rack_A01|LoadedPallet_A04|BlockPallet_A01|Bin_A06|GuardRail_A01>
    /Game/Warehouse/Meshes/SM_<WallA_6M|CeilingA_6X6|Floor02|LampCeilingA_04|BeamA_9M> (shell + lighting kit)
    /Game/Forklift/Meshes/SM_Forklift_<Body|Glass|Lift|FrontWheelL|...>
    /Game/ForkliftB/Meshes/SM_ForkliftB_<Body|Lift|DriveWheel|Swivel|RollerA..D>

Set IMPORT_ONLY=forklift_b in the environment to import just ForkliftB and leave everything else in place (delete
Content/ForkliftB on disk first: re-importing over existing assets fails), or IMPORT_ONLY=pallet to only rebuild the
loose block pallet's fork-pocket collision.

The C++ classes (Warehouse.cpp / ForkliftVehicle.cpp) load meshes from exactly these paths.
"""
import json
import os

import unreal

PROJECT = unreal.SystemLibrary.get_project_directory()
BAKED = os.path.join(PROJECT, "ExternalAssets", "baked")

tools = unreal.AssetToolsHelpers.get_asset_tools()
asset_lib = unreal.EditorAssetLibrary
mat_lib = unreal.MaterialEditingLibrary
mesh_lib = unreal.EditorStaticMeshLibrary

MATERIAL_DIR = "/Game/Warehouse/Materials"
WAREHOUSE_MESH_DIR = "/Game/Warehouse/Meshes"
FORKLIFT_MESH_DIR = "/Game/Forklift/Meshes"
FORKLIFT_B_MESH_DIR = "/Game/ForkliftB/Meshes"
TEMP_DIR = "/Game/_ImportTemp"


def log(msg):
    unreal.log(f"[import_assets] {msg}")


def fresh_asset(directory, name, asset_class, factory):
    path = f"{directory}/{name}"
    if asset_lib.does_asset_exist(path):
        asset_lib.delete_asset(path)
    return tools.create_asset(name, directory, asset_class, factory)


# ---- Master materials -----------------------------------------------------------------------------
def make_master(name, translucent):
    mat = fresh_asset(MATERIAL_DIR, name, unreal.Material, unreal.MaterialFactoryNew())

    def param(cls, pname, x, y, default):
        node = mat_lib.create_material_expression(mat, cls, x, y)
        node.set_editor_property("parameter_name", pname)
        node.set_editor_property("default_value", default)
        return node

    color = param(unreal.MaterialExpressionVectorParameter, "Color", -500, -200, unreal.LinearColor(0.5, 0.5, 0.5, 1.0))
    rough = param(unreal.MaterialExpressionScalarParameter, "Roughness", -500, 0, 0.6)
    metal = param(unreal.MaterialExpressionScalarParameter, "Metallic", -500, 150, 0.0)
    mat_lib.connect_material_property(color, "", unreal.MaterialProperty.MP_BASE_COLOR)
    mat_lib.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    mat_lib.connect_material_property(metal, "", unreal.MaterialProperty.MP_METALLIC)
    if translucent:
        mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT)
        opacity = param(unreal.MaterialExpressionScalarParameter, "Opacity", -500, 300, 0.25)
        mat_lib.connect_material_property(opacity, "", unreal.MaterialProperty.MP_OPACITY)
    mat_lib.recompile_material(mat)
    asset_lib.save_loaded_asset(mat)
    return mat


def make_instance(key, props, masters, cache):
    if key in cache:
        return cache[key]
    translucent = "opacity" in props
    name = f"MI_{key}"
    # A selective import leaves other assets in place, and those may reference a shared instance (e.g. MI_Glass),
    # which cannot be deleted and recreated - reuse it.
    if os.environ.get("IMPORT_ONLY") and asset_lib.does_asset_exist(f"{MATERIAL_DIR}/{name}"):
        cache[key] = unreal.load_asset(f"{MATERIAL_DIR}/{name}")
        return cache[key]
    mi = fresh_asset(MATERIAL_DIR, name, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    mi.set_editor_property("parent", masters["glass" if translucent else "flat"])
    r, g, b = props["color"]
    mat_lib.set_material_instance_vector_parameter_value(mi, "Color", unreal.LinearColor(r, g, b, 1.0))
    mat_lib.set_material_instance_scalar_parameter_value(mi, "Roughness", float(props["roughness"]))
    mat_lib.set_material_instance_scalar_parameter_value(mi, "Metallic", float(props["metallic"]))
    if translucent:
        mat_lib.set_material_instance_scalar_parameter_value(mi, "Opacity", float(props["opacity"]))
    mat_lib.update_material_instance(mi)
    asset_lib.save_loaded_asset(mi)
    cache[key] = mi
    return mi


def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


# ---- Import ---------------------------------------------------------------------------------------
def import_usd(usd_path, dest):
    if asset_lib.does_directory_exist(dest):
        asset_lib.delete_directory(dest)
    task = unreal.AssetImportTask()
    task.filename = usd_path
    task.destination_path = dest
    task.automated = True
    task.save = False
    task.replace_existing = True
    tools.import_asset_tasks([task])
    meshes = []
    for path in task.imported_object_paths:
        asset = unreal.load_asset(path)
        if isinstance(asset, unreal.StaticMesh):
            meshes.append(asset)
    return meshes


def assign_materials(mesh, materials_json, masters, cache):
    """Replace importer-made materials with our MIs, matching by slot / material name."""
    keys = sorted(materials_json, key=len, reverse=True)  # longest first so 'DarkGray2' beats 'DarkGray'
    for index, slot in enumerate(mesh.static_materials):
        candidates = [str(slot.material_slot_name)]
        if slot.material_interface:
            candidates.append(slot.material_interface.get_name())
        match = next((k for k in keys for c in candidates if c.endswith(k) or c == k or c.endswith(f"_{k}")), None)
        if match is None:
            log(f"  ! {mesh.get_name()}: no material match for slot {index} {candidates}")
            continue
        mesh.set_material(index, make_instance(match, materials_json[match], masters, cache))


# BlockPallet_A01 (1.0 x 1.2 x 0.12 m, mesh pivot at the centre of its underside) is a three-stringer pallet: a top deck
# (z 10-12), bottom boards (z 0-2) and three 2 cm stringers (z 2-10) at y = -60..-58, -1..1 and 58..60, so it has two
# fork pockets 57 cm wide and 8 cm tall (z 2-10) running the full 100 cm. A single bounding box would fill them, so
# the collision is these boxes instead: (centre xyz, full size xyz) in cm, measured from the USD's vertex layers.
PALLET_BOXES = [
    ((0, 0, 11), (100, 120, 2)),
    ((0, 0, 1), (100, 120, 2)),
    ((0, -59, 6), (100, 2, 8)),
    ((0, 0, 6), (100, 2, 8)),
    ((0, 59, 6), (100, 2, 8)),
]


def set_box_collision(mesh, boxes):
    """Replace the mesh's collision with the given boxes (analytic shapes, no cooking needed)."""
    mesh_lib.remove_collisions(mesh)
    body_setup = mesh.get_editor_property("body_setup")
    agg = body_setup.get_editor_property("agg_geom")
    elems = []
    for center, size in boxes:
        e = unreal.KBoxElem()
        e.set_editor_property("center", unreal.Vector(*center))
        e.set_editor_property("x", float(size[0]))
        e.set_editor_property("y", float(size[1]))
        e.set_editor_property("z", float(size[2]))
        elems.append(e)
    agg.set_editor_property("box_elems", elems)
    body_setup.set_editor_property("agg_geom", agg)
    body_setup.set_editor_property("never_needs_cooked_collision_data", False)
    mesh.modify()  # box shapes are analytic: they are saved with the asset, nothing to cook
    log(f"  {mesh.get_name()}: {len(boxes)} box colliders")


def finish_mesh(mesh, final_dir, final_name, collision, nanite=True):
    final_path = f"{final_dir}/{final_name}"
    if not asset_lib.rename_asset(mesh.get_path_name().split(".")[0], final_path):
        raise RuntimeError(f"could not rename {mesh.get_path_name()} -> {final_path}")
    mesh = unreal.load_asset(final_path)
    if mesh is None:
        raise RuntimeError(f"{final_path} did not load after rename")

    mesh_lib.remove_collisions(mesh)
    if collision == "pallet_boxes":
        set_box_collision(mesh, PALLET_BOXES)
    elif collision == "box":
        mesh_lib.add_simple_collisions(mesh, unreal.ScriptingCollisionShapeType.BOX)
    elif isinstance(collision, tuple):  # (hull_count, max_hull_verts)
        mesh_lib.set_convex_decomposition_collisions(mesh, collision[0], collision[1], 100000)

    # The USD importer creates body setups flagged "never needs cooked collision data" (it imported the
    # mesh with no collision). That flag makes UBodySetup::CreatePhysicsMeshes bail out, so convex hulls
    # we add afterwards never become physics shapes at runtime: the body has no shapes and is silently
    # discarded (mass 0, never simulates). Clear it whenever we added convex collision.
    if isinstance(collision, tuple):
        mesh.get_editor_property("body_setup").set_editor_property("never_needs_cooked_collision_data", False)

    translucent = any(
        slot.material_interface and slot.material_interface.get_base_material().get_editor_property("blend_mode") == unreal.BlendMode.BLEND_TRANSLUCENT
        for slot in mesh.static_materials
    )
    if nanite and not translucent and mesh.get_num_triangles(0) > 20000:
        settings = mesh.get_editor_property("nanite_settings")
        settings.enabled = True
        mesh.set_editor_property("nanite_settings", settings)

    b = mesh.get_bounds()
    log(f"{final_path}: size(cm)=({b.box_extent.x * 2:.1f}, {b.box_extent.y * 2:.1f}, {b.box_extent.z * 2:.1f}) "
        f"center=({b.origin.x:.1f}, {b.origin.y:.1f}, {b.origin.z:.1f}) tris={mesh.get_num_triangles(0)} "
        f"slots={[str(s.material_slot_name) for s in mesh.static_materials]}")
    asset_lib.save_loaded_asset(mesh)
    return mesh


def import_forklift_b(masters, cache):
    """ForkliftB parts. Collision: the chassis gets convex hulls; the lift is left WITHOUT mesh collision because
    it is one 270k-triangle mesh (carriage + forks + cab + inner mast) - ForkliftVehicle adds box colliders for
    the tines / carriage / platform instead. Wheels, rollers and the swivel are visual only (raycast wheels)."""
    part_collision = {
        "Body": (8, 32),
        "Lift": None,
        "DriveWheel": None,
        "Swivel": None,
        "RollerA": None,
        "RollerB": None,
        "RollerC": None,
        "RollerD": None,
    }
    for part, collision in part_collision.items():
        meshes = import_usd(os.path.join(BAKED, f"ForkliftB_{part}.baked.usd"), f"{TEMP_DIR}/ForkliftB_{part}")
        if len(meshes) != 1:
            raise RuntimeError(f"ForkliftB_{part}: expected 1 mesh, got {[m.get_name() for m in meshes]}")
        materials_json = json.load(open(os.path.join(BAKED, f"ForkliftB_{part}.materials.json")))
        assign_materials(meshes[0], materials_json, masters, cache)
        finish_mesh(meshes[0], FORKLIFT_B_MESH_DIR, f"SM_ForkliftB_{part}", collision)


def main():
    if os.environ.get("IMPORT_ONLY") == "pallet":
        pallet = unreal.load_asset(f"{WAREHOUSE_MESH_DIR}/SM_BlockPallet_A01")
        set_box_collision(pallet, PALLET_BOXES)
        asset_lib.save_loaded_asset(pallet)
        log("done (pallet collision only)")
        return

    if os.environ.get("IMPORT_ONLY") == "forklift_b":
        for d in (FORKLIFT_B_MESH_DIR, TEMP_DIR):
            if asset_lib.does_directory_exist(d):
                asset_lib.delete_directory(d)
            asset_lib.make_directory(d)
        masters = {"flat": unreal.load_asset(f"{MATERIAL_DIR}/M_FlatColor"), "glass": unreal.load_asset(f"{MATERIAL_DIR}/M_FlatGlass")}
        import_forklift_b(masters, {})
        pallet = unreal.load_asset(f"{WAREHOUSE_MESH_DIR}/SM_BlockPallet_A01")
        set_box_collision(pallet, PALLET_BOXES)
        asset_lib.save_loaded_asset(pallet)
        asset_lib.delete_directory(TEMP_DIR)
        asset_lib.save_directory("/Game/Warehouse", only_if_is_dirty=True, recursive=True)
        asset_lib.save_directory("/Game/ForkliftB", only_if_is_dirty=False, recursive=True)
        log("done (forklift_b only)")
        return

    # Start from a clean slate: re-imports are idempotent, and renaming onto an existing asset in the
    # same session is unreliable.
    for d in (MATERIAL_DIR, WAREHOUSE_MESH_DIR, FORKLIFT_MESH_DIR, FORKLIFT_B_MESH_DIR, TEMP_DIR):
        if asset_lib.does_directory_exist(d):
            asset_lib.delete_directory(d)
        asset_lib.make_directory(d)

    masters = {"flat": make_master("M_FlatColor", False), "glass": make_master("M_FlatGlass", True)}
    cache = {}

    # Warehouse props: (baked name, collision). Racks get their proxy box in C++; the rest get a box.
    warehouse = {
        "Rack_A01": None,
        "LoadedPallet_A04": "box",
        "BlockPallet_A01": "pallet_boxes",
        "Bin_A06": "box",
        "GuardRail_A01": "box",
        # Shell/lighting kit: floor and walls need real collision (the forklift drives on/against them);
        # the ceiling tile and beam are up at 9 m and the lamp hangs below that - nothing plays up there,
        # so they're visual-only like Rack_A01.
        "WallA_6M": "box",
        "CeilingA_6X6": "box",
        "Floor02": "box",
        "LampCeilingA_04": None,
        "BeamA_9M": None,
        # Real rack kit: uprights/shelves/end-shields get collision (forklift and sensors interact with
        # the rack run); the stock on the shelves is visual clutter like Rack_A01 was.
        "RackFrame_03": "box",
        "RackShelf_01": "box",
        "Rackshield_02": "box",
        "PaletteA_01": None,
        "CardBoxA_01": None,
        "CardBoxA_02": None,
    }
    for name, collision in warehouse.items():
        usd = os.path.join(BAKED, f"{name}.baked.usd")
        meshes = import_usd(usd, f"{TEMP_DIR}/{name}")
        if len(meshes) != 1:
            raise RuntimeError(f"{name}: expected 1 mesh, got {[m.get_name() for m in meshes]}")
        materials_json = json.load(open(os.path.join(BAKED, f"{name}.materials.json")))
        assign_materials(meshes[0], materials_json, masters, cache)
        finish_mesh(meshes[0], WAREHOUSE_MESH_DIR, f"SM_{name}", collision)

    # Forklift parts: convex collision mirrors forklift_c.usd (convexDecomposition body/mast,
    # convexHull rotators; the wheels are USD Cylinders, a single hull is the closest UE shape).
    part_collision = {
        "Body": (8, 32),
        "Glass": None,
        "Lift": (6, 32),
        "FrontWheelL": (1, 48),
        "FrontWheelR": (1, 48),
        "RearWheelL": (1, 48),
        "RearWheelR": (1, 48),
        "RotatorL": (1, 32),
        "RotatorR": (1, 32),
    }
    for part, collision in part_collision.items():
        meshes = import_usd(os.path.join(BAKED, f"Forklift_{part}.baked.usd"), f"{TEMP_DIR}/Forklift_{part}")
        if len(meshes) != 1:
            raise RuntimeError(f"Forklift_{part}: expected 1 mesh, got {[m.get_name() for m in meshes]}")
        materials_json = json.load(open(os.path.join(BAKED, f"Forklift_{part}.materials.json")))
        assign_materials(meshes[0], materials_json, masters, cache)
        finish_mesh(meshes[0], FORKLIFT_MESH_DIR, f"SM_Forklift_{part}", collision)

    import_forklift_b(masters, cache)

    asset_lib.delete_directory(TEMP_DIR)
    asset_lib.save_directory("/Game/Warehouse", only_if_is_dirty=False, recursive=True)
    asset_lib.save_directory("/Game/Forklift", only_if_is_dirty=False, recursive=True)
    asset_lib.save_directory("/Game/ForkliftB", only_if_is_dirty=False, recursive=True)
    log("done")


main()
