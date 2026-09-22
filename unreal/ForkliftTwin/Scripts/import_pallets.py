"""Import the baked pallets into /Game/Pallets (run inside UE via -ExecutePythonScript, after bake_pallets.py).

Creates
    /Game/Pallets/Materials/M_PalletFlat, M_PalletGlass and one MI_<slot> per material
    /Game/Pallets/Meshes/SM_<asset>                     one static mesh per usable asset, pivot at the bottom centre
and writes Content/Pallets/catalog.json: the bake's catalogue plus each asset's mesh path, which the C++ data
generator (SyntheticDataGenerator) reads at run time.

Independent of import_assets.py on purpose: it wipes /Game/Warehouse (including its master materials) on every
run, so nothing here references that folder.
"""
import json
import os

import unreal

PROJECT = unreal.SystemLibrary.get_project_directory()
CONTENT = unreal.SystemLibrary.get_project_content_directory()
BAKED = os.path.join(PROJECT, "ExternalAssets", "pallets_baked")

tools = unreal.AssetToolsHelpers.get_asset_tools()
asset_lib = unreal.EditorAssetLibrary
mat_lib = unreal.MaterialEditingLibrary
mesh_lib = unreal.EditorStaticMeshLibrary

ROOT_DIR = "/Game/Pallets"
MATERIAL_DIR = f"{ROOT_DIR}/Materials"
MESH_DIR = f"{ROOT_DIR}/Meshes"
TEMP_DIR = "/Game/_PalletImportTemp"


def log(msg):
    unreal.log(f"[import_pallets] {msg}")


def fresh_asset(directory, name, asset_class, factory):
    path = f"{directory}/{name}"
    if asset_lib.does_asset_exist(path):
        asset_lib.delete_asset(path)
    return tools.create_asset(name, directory, asset_class, factory)


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
        opacity = param(unreal.MaterialExpressionScalarParameter, "Opacity", -500, 300, 0.12)
        mat_lib.connect_material_property(opacity, "", unreal.MaterialProperty.MP_OPACITY)
    mat_lib.recompile_material(mat)
    asset_lib.save_loaded_asset(mat)
    return mat


def make_instance(key, props, masters, cache):
    if key in cache:
        return cache[key]
    translucent = "opacity" in props
    mi = fresh_asset(MATERIAL_DIR, f"MI_{key}", unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
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
    return [a for a in (unreal.load_asset(p) for p in task.imported_object_paths) if isinstance(a, unreal.StaticMesh)]


def assign_materials(mesh, materials_json, masters, cache):
    keys = sorted(materials_json, key=len, reverse=True)  # longest first
    slots = []
    for index, slot in enumerate(mesh.static_materials):
        candidates = [str(slot.material_slot_name)]
        if slot.material_interface:
            candidates.append(slot.material_interface.get_name())
        match = next((k for k in keys for c in candidates if c.endswith(k) or c == k or c.endswith(f"_{k}")), None)
        if match is None:
            log(f"  ! {mesh.get_name()}: no material match for slot {index} {candidates}")
            continue
        mesh.set_material(index, make_instance(match, materials_json[match], masters, cache))
        slots.append(dict(index=index, key=match))
    return slots


def finish_mesh(mesh, name):
    final_path = f"{MESH_DIR}/SM_{name}"
    if not asset_lib.rename_asset(mesh.get_path_name().split(".")[0], final_path):
        raise RuntimeError(f"could not rename {mesh.get_path_name()} -> {final_path}")
    mesh = unreal.load_asset(final_path)
    mesh_lib.remove_collisions(mesh)
    mesh_lib.add_simple_collisions(mesh, unreal.ScriptingCollisionShapeType.BOX)
    if mesh.get_num_triangles(0) > 20000:
        settings = mesh.get_editor_property("nanite_settings")
        settings.enabled = True
        mesh.set_editor_property("nanite_settings", settings)
    asset_lib.save_loaded_asset(mesh)
    return mesh


def main():
    catalog = json.load(open(os.path.join(BAKED, "catalog.json")))
    for d in (ROOT_DIR, TEMP_DIR):
        if asset_lib.does_directory_exist(d):
            asset_lib.delete_directory(d)
    for d in (MATERIAL_DIR, MESH_DIR):
        asset_lib.make_directory(d)

    masters = {"flat": make_master("M_PalletFlat", False), "glass": make_master("M_PalletGlass", True)}
    cache = {}
    bad = []
    everything = catalog["assets"] + catalog["props"]  # pallets / loaded pallets, then totes and bins
    for n, info in enumerate(everything, 1):
        name = info["name"]
        try:
            meshes = import_usd(os.path.join(BAKED, f"{name}.baked.usd"), f"{TEMP_DIR}/{name}")
            if len(meshes) != 1:
                raise RuntimeError(f"expected 1 mesh, got {[m.get_name() for m in meshes]}")
            materials_json = json.load(open(os.path.join(BAKED, f"{name}.materials.json")))
            info["material_slots"] = assign_materials(meshes[0], materials_json, masters, cache)
            mesh = finish_mesh(meshes[0], name)
            info["mesh"] = f"{MESH_DIR}/SM_{name}.SM_{name}"

            # The bake put the pivot at the bottom centre and Y-flipped into UE's frame; check the importer agrees.
            b = mesh.get_bounds()
            lo, hi = info["bbox_cm"]["lo"], info["bbox_cm"]["hi"]
            want = [(lo[i] + hi[i]) / 2 for i in range(3)]
            got = [b.origin.x, b.origin.y, b.origin.z]
            err = max(abs(w - g) for w, g in zip(want, got))
            ext_err = max(abs((hi[i] - lo[i]) / 2 - e) for i, e in enumerate([b.box_extent.x, b.box_extent.y, b.box_extent.z]))
            flag = "" if err < 1.5 and ext_err < 1.5 else "   <-- BOUNDS MISMATCH"
            if flag:
                bad.append(name)
            log(f"[{n}/{len(everything)}] {name}: tris={mesh.get_num_triangles(0)} centre err {err:.2f} cm, extent err {ext_err:.2f} cm{flag}")
        except Exception as exc:  # keep going; the asset is dropped from the runtime catalogue
            log(f"  ! {name} FAILED: {exc!r}")
            info["import_failed"] = repr(exc)
            bad.append(name)

    catalog["assets"] = [a for a in catalog["assets"] if "mesh" in a and "import_failed" not in a]
    catalog["props"] = [a for a in catalog["props"] if "mesh" in a and "import_failed" not in a]
    out_dir = os.path.join(CONTENT, "Pallets")
    os.makedirs(out_dir, exist_ok=True)
    json.dump(catalog, open(os.path.join(out_dir, "catalog.json"), "w"), indent=1)
    if asset_lib.does_directory_exist(TEMP_DIR):
        asset_lib.delete_directory(TEMP_DIR)
    asset_lib.save_directory(ROOT_DIR, only_if_is_dirty=False, recursive=True)
    log(f"done: {len(catalog['assets'])} pallets + {len(catalog['props'])} totes imported; problems: {bad if bad else 'none'}")


main()
