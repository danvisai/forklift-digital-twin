"""Bake the NVIDIA USD assets into UE-friendly USD files (run with system Python, not inside UE).

Why this exists: UE's USD importer keeps each mesh prim's vertex data in its *local* frame and
carries the prim transforms on actors, and it cannot read the MDL materials / UDIM atlas. The
forklift is 49 meshes across 8 articulated parts, so importing it directly gives 49 loose meshes
that are wrongly scaled and rotated relative to each other. Instead this script:

  * flattens every mesh of a part into ONE mesh, in the shared root frame (so every part keeps its
    pivot at the forklift origin and joints can be placed at their real coordinates),
  * triangulates, and un-shares vertices so normals/UVs survive as-is,
  * resolves each triangle to a material *slot* - by material name for the warehouse props, and by
    sampling the forklift's albedo/ORM atlas for colour, roughness and metallic,
  * writes <name>.baked.usd plus <name>.materials.json (slot -> PBR constants) for import_assets.py.

Requires: pip install usd-core numpy pillow
"""
import json
import pathlib
import sys

import numpy as np
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

Image.MAX_IMAGE_PIXELS = None

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "ExternalAssets"
OUT = SRC / "baked"

# ---- Warehouse: material name -> PBR constants ---------------------------------------------------
# The MDL sources aren't consumable in UE, so these are hand-picked approximations of the look of
# NVIDIA's materials (sRGB-ish base colour; UE treats the Color param as linear so they are
# converted in import_assets.py).
WAREHOUSE_MATERIALS = {
    "Wood_New_A": dict(color=(0.62, 0.45, 0.27), roughness=0.75, metallic=0.0),
    "Wood_Recycled_A": dict(color=(0.50, 0.38, 0.25), roughness=0.85, metallic=0.0),
    "Metal_Glossy_A": dict(color=(0.62, 0.63, 0.65), roughness=0.30, metallic=1.0),
    "Metal_Glossy_A_Rack_A": dict(color=(0.62, 0.63, 0.65), roughness=0.30, metallic=1.0),
    "Metal_Painted_Blue_B_Rack_A": dict(color=(0.06, 0.22, 0.62), roughness=0.45, metallic=0.5),
    "Plastic_Bin": dict(color=(0.30, 0.32, 0.35), roughness=0.55, metallic=0.0),
    "Pallet_Assembly_Tile": dict(color=(0.66, 0.52, 0.34), roughness=0.9, metallic=0.0),
    "Pallet_Assembly_Trim": dict(color=(0.78, 0.78, 0.74), roughness=0.6, metallic=0.0),
    "Plastic_Wrap_Transparent": dict(color=(0.92, 0.96, 1.0), roughness=0.15, metallic=0.0, opacity=0.12),
    # GuardRail subsets carry no material binding; keyed by subset name.
    "M_GuardRail_A01_Body": dict(color=(0.95, 0.72, 0.05), roughness=0.5, metallic=0.1),
    "M_GuardRail_A01_Cap": dict(color=(0.10, 0.10, 0.11), roughness=0.6, metallic=0.0),
    "M_GuardRail_A01_Joints": dict(color=(0.45, 0.46, 0.48), roughness=0.4, metallic=1.0),
    # Shell/lighting kit (from Isaac Sim's own Simple_Warehouse/warehouse.usd). Keyed by the exact bound
    # material prim name (not the mesh's own name) since these meshes bind materials directly rather than
    # through GeomSubsets. Colours are linear averages sampled from the real T_*_D.png / T_*_ORM.png
    # textures (see the material's AlbedoTexture / MergeMapInput inputs) - not hand-picked like the rest
    # of this dict - so they track NVIDIA's actual look instead of a guess.
    "MI_WallA_01": dict(color=(0.472, 0.472, 0.472), roughness=0.40, metallic=0.0),   # SM_WallA_6M body
    "MI_WallB_01": dict(color=(0.341, 0.345, 0.349), roughness=0.30, metallic=0.65),  # SM_CeilingA_6X6 (underside)
    "MI_Floor_01": dict(color=(0.051, 0.050, 0.049), roughness=0.30, metallic=0.0),   # SM_floor02
    "MI_BeamsA_01": dict(color=(0.124, 0.125, 0.120), roughness=0.48, metallic=0.06), # SM_BeamA_9M
    "MI_BeamsA_02": dict(color=(0.124, 0.125, 0.120), roughness=0.48, metallic=0.06), # SM_WallA_6M trim
    "MI_LampCeilingA": dict(color=(0.251, 0.247, 0.233), roughness=0.47, metallic=0.03),
    "M_Glow": dict(color=(0.85, 0.85, 0.80), roughness=0.25, metallic=0.0),  # lamp lens; no source texture
    # Real pallet-rack kit (SM_RackFrame_03 / SM_RackShelf_01 / SM_Rackshield_02) plus what stocks it.
    # RackSetA_01's own D-texture samples as flat neutral grey (~0.42 sRGB) - the MDL shader's
    # ColorAlbedo (0.488, 0.106, 0) is what actually paints the beam orange, so that one is srgb-decoded
    # and used directly instead of the (colourless) texture average; every other entry here is a genuine
    # sampled texture average, same method as the shell kit above.
    "MI_RackSetA_01": dict(color=(0.195, 0.012, 0.0), roughness=0.40, metallic=0.03),   # RackShelf beam (orange)
    "MI_RackSetB_01": dict(color=(0.138, 0.137, 0.134), roughness=0.49, metallic=0.0),  # RackFrame upright
    "MI_RackSetC_01": dict(color=(0.132, 0.134, 0.134), roughness=0.32, metallic=0.0),  # RackFrame/RackShelf trim
    "MI_RackSetD_01": dict(color=(0.269, 0.270, 0.270), roughness=0.21, metallic=0.0),  # RackShelf underside
    "MI_RackShield_01": dict(color=(0.677, 0.316, 0.015), roughness=0.50, metallic=0.07),
    "MI_PaletteSmallA_01": dict(color=(0.204, 0.146, 0.111), roughness=0.86, metallic=0.0),
    "MI_CardBoxA": dict(color=(0.246, 0.166, 0.098), roughness=0.85, metallic=0.0),
}
FALLBACK_MATERIAL = dict(color=(0.6, 0.6, 0.6), roughness=0.6, metallic=0.0)

# (usd file, output name, unreal asset name)
WAREHOUSE_ASSETS = [
    ("Rack_A01_PR_NVD_01.usd", "Rack_A01"),
    ("Pallet_Asm_A04_120x122x75cm_PR_V_NVD_01.usd", "LoadedPallet_A04"),
    ("BlockPallet_A01_PR_NVD_01.usd", "BlockPallet_A01"),
    ("Bin_A06_30x51x19cm_PR_V_NVD_01.usd", "Bin_A06"),
    ("GuardRail_A01_01.usd", "GuardRail_A01"),
    # Shell/lighting kit: real modular tiles instead of flat-colour boxes. See Warehouse.cpp for how
    # they're tiled across the 36 x 24 m shell at the kit's true 9 m (3 x 3 m tier) wall height.
    ("SM_WallA_6M.usd", "WallA_6M"),
    ("SM_CeilingA_6X6.usd", "CeilingA_6X6"),
    ("SM_floor02.usd", "Floor02"),
    ("SM_LampCeilingA_04.usd", "LampCeilingA_04"),
    ("SM_BeamA_9M.usd", "BeamA_9M"),
    # Real modular pallet-rack kit + stock (replaces the DigitalTwin Rack_A01 for the main racking).
    ("SM_RackFrame_03.usd", "RackFrame_03"),
    ("SM_RackShelf_01.usd", "RackShelf_01"),
    ("SM_Rackshield_02.usd", "Rackshield_02"),
    ("SM_PaletteA_01.usd", "PaletteA_01"),
    ("SM_CardBoxA_01.usd", "CardBoxA_01"),
    ("SM_CardBoxA_02.usd", "CardBoxA_02"),
]

# ---- Forklift: articulated parts -> output mesh names ---------------------------------------------
# Left/right keep their meaning across the USD->UE handedness flip (USD +Y is the truck's left).
FORKLIFT_PARTS = {
    "body": "Body",
    "lift": "Lift",
    "left_front_wheel": "FrontWheelL",
    "right_front_wheel": "FrontWheelR",
    "left_back_wheel": "RearWheelL",
    "right_back_wheel": "RearWheelR",
    "left_rotator": "RotatorL",
    "right_rotator": "RotatorR",
}
# The albedo atlas is greyscale; paint colour comes from the tint on the asset's own material variants
# (M_Forklift_C01_Yellow: diffuse_tint = (0.919, 0.484, 0.032)). Atlas regions brighter than
# PAINT_BRIGHTNESS are the paintable panels, so they get that tint; darker regions (rubber, steel,
# seat) keep their sampled colour.
PAINT_TINT_SRGB = (0.9189189, 0.48398682, 0.03193154)
PAINT_BRIGHTNESS = 0.55
GLASS_KEY = "Glass"
GLASS_MATERIAL = dict(color=(0.55, 0.70, 0.78), roughness=0.05, metallic=0.0, opacity=0.25)


# ---- Mesh flattening --------------------------------------------------------------------------------
class Baked:
    """Triangle soup (3 unshared vertices per triangle) with a material key per triangle."""

    def __init__(self):
        self.points, self.normals, self.uvs, self.keys = [], [], [], []

    def add(self, points, normals, uvs, keys):
        self.points.append(points)
        self.normals.append(normals)
        self.uvs.append(uvs)
        self.keys.extend(keys)

    def arrays(self):
        return (np.concatenate(self.points), np.concatenate(self.normals), np.concatenate(self.uvs), np.array(self.keys))


def primvar_values(mesh, name, n_corners, fvi):
    """Return a per-corner array for a primvar (handles indexed and all interpolations), or None."""
    pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).GetPrimvar(name)
    if not pv or not pv.HasValue():
        return None
    vals = np.array(pv.Get(), dtype=np.float64)
    idx = pv.GetIndices()
    if idx is not None and len(idx):
        vals = vals[np.array(idx)]
    interp = pv.GetInterpolation()
    if interp == UsdGeom.Tokens.faceVarying:
        return vals
    if interp == UsdGeom.Tokens.vertex or interp == UsdGeom.Tokens.varying:
        return vals[fvi]
    return None


def flatten_mesh(prim, xf_cache, face_slot_of):
    """Triangulate one mesh prim into world space. Returns (points[T*3,3], normals, uvs, corner_face[T]) ."""
    mesh = UsdGeom.Mesh(prim)
    counts = np.array(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    fvi = np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)

    m = xf_cache.GetLocalToWorldTransform(prim)
    mat = np.array(m).T  # pxr matrices are row-vector; transpose for column-vector maths
    lin = mat[:3, :3]
    world_pts = pts @ lin.T + mat[:3, 3]

    # Fan-triangulate every face.
    tri_per_face = np.maximum(counts - 2, 0)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    face_of_tri = np.repeat(np.arange(len(counts)), tri_per_face)
    within = np.arange(tri_per_face.sum()) - np.repeat(np.cumsum(tri_per_face) - tri_per_face, tri_per_face)
    a = starts[face_of_tri]
    corners = np.stack([a, a + within + 1, a + within + 2], axis=1)  # [T,3] corner ids

    flip = (mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded) ^ (np.linalg.det(lin) < 0)
    if flip:
        corners = corners[:, [0, 2, 1]]

    tri_pts = world_pts[fvi[corners]]  # [T,3,3]

    normals = primvar_values(mesh, "normals", len(fvi), fvi)
    if normals is None and mesh.GetNormalsAttr().HasValue():
        n = np.array(mesh.GetNormalsAttr().Get(), dtype=np.float64)
        interp = mesh.GetNormalsInterpolation()
        normals = n if interp == UsdGeom.Tokens.faceVarying else (n[fvi] if interp == UsdGeom.Tokens.vertex else None)
    if normals is not None:
        nlin = np.linalg.inv(lin).T
        tri_n = (normals @ nlin.T)[corners]
        tri_n /= np.maximum(np.linalg.norm(tri_n, axis=-1, keepdims=True), 1e-12)
    else:
        e1, e2 = tri_pts[:, 1] - tri_pts[:, 0], tri_pts[:, 2] - tri_pts[:, 0]
        fn = np.cross(e1, e2)
        fn /= np.maximum(np.linalg.norm(fn, axis=-1, keepdims=True), 1e-12)
        tri_n = np.repeat(fn[:, None, :], 3, axis=1)

    uv = primvar_values(mesh, "st", len(fvi), fvi)
    tri_uv = uv[corners] if uv is not None else np.zeros((len(corners), 3, 2))
    return tri_pts, tri_n, tri_uv, face_of_tri


def bound_material_name(prim):
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    return mat.GetPath().name if mat else None


def face_keys_for_mesh(prim, n_faces):
    """Material key per source face, from GeomSubsets (else the mesh's bound material)."""
    default = bound_material_name(prim) or prim.GetName()
    keys = np.full(n_faces, default, dtype=object)
    for subset in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Mesh(prim)):
        name = bound_material_name(subset.GetPrim()) or subset.GetPrim().GetName()
        idx = np.array(subset.GetIndicesAttr().Get(), dtype=np.int64)
        keys[idx] = name
    return keys


# ---- Output -------------------------------------------------------------------------------------------
def write_usd(out_path, mpu, meshes, materials):
    """meshes: {name: (pts, nrm, uv, keys)}; materials: {key: dict(color, roughness, metallic, opacity?)}"""
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, mpu)
    root = UsdGeom.Xform.Define(stage, "/Baked")
    stage.SetDefaultPrim(root.GetPrim())

    mat_prims = {}
    for key, props in materials.items():
        mat = UsdShade.Material.Define(stage, f"/Baked/Looks/{key}")
        shader = UsdShade.Shader.Define(stage, f"/Baked/Looks/{key}/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*props["color"]))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(props["roughness"]))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(props["metallic"]))
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        mat_prims[key] = mat

    for name, (pts, nrm, uv, keys) in meshes.items():
        tri_count = len(keys)
        mesh = UsdGeom.Mesh.Define(stage, f"/Baked/{name}")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.reshape(-1, 3).astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(tri_count, 3, dtype=np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(tri_count * 3, dtype=np.int32)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(nrm.reshape(-1, 3).astype(np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr("none")
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray.FromNumpy(uv.reshape(-1, 2).astype(np.float32)))
        for key in sorted(set(keys.tolist())):
            idx = np.nonzero(keys == key)[0].astype(np.int32)
            subset = UsdGeom.Subset.CreateGeomSubset(mesh, key, UsdGeom.Tokens.face, Vt.IntArray.FromNumpy(idx), "materialBind")
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat_prims[key])
    stage.GetRootLayer().Save()


# ---- Warehouse ----------------------------------------------------------------------------------------
def bake_warehouse(usd_name, out_name):
    stage = Usd.Stage.Open(str(SRC / usd_name))
    xf_cache = UsdGeom.XformCache()
    baked = Baked()
    used = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(prim).ComputePurpose() not in ("default", "render"):
            continue
        n_faces = len(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get())
        pts, nrm, uv, face_of_tri = flatten_mesh(prim, xf_cache, None)
        face_keys = face_keys_for_mesh(prim, n_faces)
        keys = face_keys[face_of_tri]
        baked.add(pts, nrm, uv, keys)
    pts, nrm, uv, keys = baked.arrays()
    for k in set(keys.tolist()):
        used[k] = WAREHOUSE_MATERIALS.get(k, FALLBACK_MATERIAL)
        if k not in WAREHOUSE_MATERIALS:
            print(f"  ! {out_name}: no material entry for '{k}', using fallback")
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    write_usd(OUT / f"{out_name}.baked.usd", mpu, {out_name: (pts, nrm, uv, keys)}, used)
    (OUT / f"{out_name}.materials.json").write_text(json.dumps(used, indent=2))
    print(f"baked {out_name}: {len(keys)} tris, materials {sorted(used)}")


# ---- Forklift -----------------------------------------------------------------------------------------
class Atlas:
    """Downsampled UDIM atlas; sample(uv) -> RGB in 0..1."""

    def __init__(self, kind, prefix="T_Forklift_C01", tile_ids=(1001, 1002, 1003)):
        self.tiles = {}
        for tile in tile_ids:
            path = SRC / "textures" / f"{prefix}_{kind}.{tile}.png"
            if path.exists():
                img = Image.open(path).convert("RGB")
                img = img.reduce(max(1, img.width // 512))
                self.tiles[tile] = np.asarray(img, dtype=np.float64) / 255.0

    def sample(self, uv):
        out = np.full((len(uv), 3), np.nan)
        tile_id = 1001 + np.floor(uv[:, 0]).astype(int) + 10 * np.floor(uv[:, 1]).astype(int)
        for tile, tex in self.tiles.items():
            sel = tile_id == tile
            if not sel.any():
                continue
            h, w, _ = tex.shape
            x = np.clip((uv[sel, 0] % 1.0) * w, 0, w - 1).astype(int)
            y = np.clip((1.0 - (uv[sel, 1] % 1.0)) * h, 0, h - 1).astype(int)
            out[sel] = tex[y, x]
        return out


def srgb_to_linear(c):
    c = np.asarray(c, dtype=np.float64)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def color_name(rgb):
    r, g, b = rgb
    mx, mn = max(rgb), min(rgb)
    if mx < 0.12:
        return "Black"
    if mx - mn < 0.10:
        return "DarkGray" if mx < 0.45 else "LightGray"
    if r > 0.6 and g > 0.45 and b < 0.3:
        return "Yellow"
    if r > g and r > b:
        return "Red" if g < 0.4 else "Orange"
    if b > r and b >= g:
        return "Blue"
    return "Green"


def bake_forklift():
    stage = Usd.Stage.Open(str(SRC / "forklift_c.usd"))
    root = stage.GetDefaultPrim()
    xf_cache = UsdGeom.XformCache()
    albedo, orm = Atlas("Albedo"), Atlas("ORM")

    # Pass 1: flatten each part, remember per-mesh sampled appearance.
    part_tris = {}    # out name -> list of (pts, nrm, uv, mesh_id)
    mesh_stats = {}   # mesh_id -> dict(albedo, rough, metal, tris)
    mesh_id = 0
    for usd_name, out_name in FORKLIFT_PARTS.items():
        for prim in Usd.PrimRange(root.GetChild(usd_name)):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            name = prim.GetName().lower()
            if "decals" in name:
                continue  # stickers: z-fight with the body and need an opacity mask we don't carry over
            pts, nrm, uv, _ = flatten_mesh(prim, xf_cache, None)
            target = "Glass" if "glass" in name else out_name
            centroid_uv = uv.mean(axis=1)
            col = albedo.sample(centroid_uv)
            orm_s = orm.sample(centroid_uv)
            ok = ~np.isnan(col[:, 0])
            mesh_stats[mesh_id] = dict(
                albedo=col[ok].mean(axis=0) if ok.any() else np.array([0.5, 0.5, 0.5]),
                rough=float(orm_s[ok, 1].mean()) if ok.any() else 0.6,
                metal=float(orm_s[ok, 2].mean()) if ok.any() else 0.0,
                tris=len(pts),
            )
            part_tris.setdefault(target, []).append((pts, nrm, uv, mesh_id))
            mesh_id += 1

    # Pass 2: cluster mesh colours into a small shared palette (greedy, biggest meshes first).
    clusters = []  # dict(color, members)
    order = sorted((mid for t in part_tris.values() for *_, mid in t if mid in mesh_stats), key=lambda m: -mesh_stats[m]["tris"])
    glass_ids = {mid for *_, mid in part_tris.get("Glass", [])}
    for mid in order:
        if mid in glass_ids:
            continue
        col = mesh_stats[mid]["albedo"]
        for cl in clusters:
            if np.linalg.norm(cl["color"] - col) < 0.12:
                cl["members"].append(mid)
                break
        else:
            clusters.append(dict(color=col.copy(), members=[mid]))
    mesh_key, materials = {}, {}
    for cl in clusters:
        w = np.array([mesh_stats[m]["tris"] for m in cl["members"]], dtype=np.float64)
        srgb = np.average([mesh_stats[m]["albedo"] for m in cl["members"]], axis=0, weights=w)
        painted = float(np.mean(srgb)) >= PAINT_BRIGHTNESS
        if painted:
            srgb = np.clip(srgb, 0.0, 1.0)
            base = "PaintYellow"
        else:
            base = color_name(srgb)
        key, n = base, 2
        while key in materials:
            key, n = f"{base}{n}", n + 1
        linear = srgb_to_linear(srgb)
        if painted:
            linear = linear * srgb_to_linear(PAINT_TINT_SRGB)
        materials[key] = dict(
            color=[float(x) for x in linear],
            roughness=float(np.clip(np.average([mesh_stats[m]["rough"] for m in cl["members"]], weights=w), 0.05, 1.0)),
            metallic=float(np.clip(np.average([mesh_stats[m]["metal"] for m in cl["members"]], weights=w), 0.0, 1.0)),
        )
        for m in cl["members"]:
            mesh_key[m] = key
    for mid in glass_ids:
        mesh_key[mid] = GLASS_KEY
    if glass_ids:
        materials[GLASS_KEY] = dict(color=list(srgb_to_linear(GLASS_MATERIAL["color"])), roughness=GLASS_MATERIAL["roughness"],
                                    metallic=GLASS_MATERIAL["metallic"], opacity=GLASS_MATERIAL["opacity"])

    # One USD file per part: UE's importer collapses every mesh under a shared root into a single
    # static mesh, so parts must live in separate files to import as separate meshes.
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    for out_name, tris in part_tris.items():
        pts = np.concatenate([t[0] for t in tris])
        nrm = np.concatenate([t[1] for t in tris])
        uv = np.concatenate([t[2] for t in tris])
        keys = np.concatenate([np.full(len(t[0]), mesh_key[t[3]], dtype=object) for t in tris])
        used = {k: materials[k] for k in sorted(set(keys.tolist()))}
        write_usd(OUT / f"Forklift_{out_name}.baked.usd", mpu, {out_name: (pts, nrm, uv, keys)}, used)
        (OUT / f"Forklift_{out_name}.materials.json").write_text(json.dumps(used, indent=2))
        print(f"  part {out_name:12s} {len(keys):7d} tris  slots={sorted(used)}")
    print("baked Forklift; palette:", {k: [round(c, 2) for c in v["color"]] for k, v in materials.items()})


# ---- Forklift B -----------------------------------------------------------------------------------------
# forklift_b.usd is a stand-on stacker: one steered drive wheel at the +X end, four small load rollers on
# the outrigger legs, and forks + operator platform + inner mast in ONE `lift` mesh. Its forks point toward
# -X; every part is rotated 180 deg about Z here so the forks lead along +X like the rest of the project.
# Out names are what import_assets.py / ForkliftVehicle.cpp look up (SM_ForkliftB_<name>).
FORKLIFT_B_PARTS = {
    "body": "Body",
    "lift": "Lift",
    "back_wheel": "DriveWheel",
    "back_wheel_swivel": "Swivel",
    "roller_front_left": "RollerA",
    "roller_front_right": "RollerB",
    "roller_back_left": "RollerC",
    "roller_back_right": "RollerD",
}
FORKLIFT_B_TILES = tuple(range(1001, 1009))
FORKLIFT_B_PALETTE_SIZE = 12   # colour clusters shared by the whole truck
FORKLIFT_B_QUANT = 0.07        # sRGB bin size used to seed the clusters


def cluster_colors(colors, k, quant):
    """Seed clusters from the k most-populated quantised colour bins, then assign every sample to the nearest seed."""
    bins = np.round(colors / quant).astype(np.int64)
    uniq, counts = np.unique(bins, axis=0, return_counts=True)
    seeds = uniq[np.argsort(-counts)[:k]] * quant
    labels = np.empty(len(colors), dtype=np.int64)
    for start in range(0, len(colors), 200000):
        chunk = colors[start:start + 200000]
        labels[start:start + 200000] = np.argmin(((chunk[:, None, :] - seeds[None]) ** 2).sum(-1), axis=1)
    return labels


def bake_forklift_b():
    stage = Usd.Stage.Open(str(SRC / "forklift_b.usd"))
    root = stage.GetDefaultPrim()
    xf_cache = UsdGeom.XformCache()
    albedo = Atlas("Albedo", "T_Forklift_B1", FORKLIFT_B_TILES)
    orm = Atlas("ORM", "T_Forklift_B1", FORKLIFT_B_TILES)
    if not albedo.tiles:
        raise RuntimeError("no Forklift_B textures in ExternalAssets/textures - run download_assets.py")

    # Pass 1: flatten every mesh of each part into the shared root frame, turned 180 deg about Z.
    parts = {}
    for usd_name, out_name in FORKLIFT_B_PARTS.items():
        for prim in Usd.PrimRange(root.GetChild(usd_name)):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            name = prim.GetName().lower()
            if "decal" in name:
                continue  # stickers: z-fight with the panels and need an opacity mask we don't carry over
            pts, nrm, uv, _ = flatten_mesh(prim, xf_cache, None)
            pts[..., :2] *= -1.0
            nrm[..., :2] *= -1.0
            parts.setdefault(out_name, []).append(dict(pts=pts, nrm=nrm, uv=uv, glass="glass" in name))

    # Pass 2: one colour / roughness / metallic sample per TRIANGLE (B merges most of the truck into a few
    # huge meshes, so per-mesh averages would paint them a single flat colour), then a shared palette.
    for tris_list in parts.values():
        for t in tris_list:
            centroid = t["uv"].mean(axis=1)
            col, orm_s = albedo.sample(centroid), orm.sample(centroid)
            bad = np.isnan(col[:, 0])
            col[bad] = 0.5
            orm_s[np.isnan(orm_s[:, 0])] = [1.0, 0.6, 0.0]
            t["col"], t["rough"], t["metal"] = col, orm_s[:, 1], orm_s[:, 2]

    solid = [t for tl in parts.values() for t in tl if not t["glass"]]
    all_col = np.concatenate([t["col"] for t in solid])
    labels = cluster_colors(all_col, FORKLIFT_B_PALETTE_SIZE, FORKLIFT_B_QUANT)
    all_rough = np.concatenate([t["rough"] for t in solid])
    all_metal = np.concatenate([t["metal"] for t in solid])
    materials, key_of_label = {}, {}
    for label in np.unique(labels):
        sel = labels == label
        srgb = np.clip(all_col[sel].mean(axis=0), 0.0, 1.0)
        # Prefixed: material instances are shared by key across all forklift parts in UE, and C already owns 'DarkGray' etc.
        base, n = "FB_" + color_name(srgb), 2
        key = base
        while key in materials:
            key, n = f"{base}{n}", n + 1
        materials[key] = dict(color=[float(x) for x in srgb_to_linear(srgb)],
                              roughness=float(np.clip(all_rough[sel].mean(), 0.05, 1.0)),
                              metallic=float(np.clip(all_metal[sel].mean(), 0.0, 1.0)))
        key_of_label[label] = key
        print(f"  palette {key:10s} {sel.sum():7d} tris  srgb={np.round(srgb, 2).tolist()}  rough={materials[key]['roughness']:.2f} metal={materials[key]['metallic']:.2f}")
    materials[GLASS_KEY] = dict(color=list(srgb_to_linear(GLASS_MATERIAL["color"])), roughness=GLASS_MATERIAL["roughness"],
                                metallic=GLASS_MATERIAL["metallic"], opacity=GLASS_MATERIAL["opacity"])

    cursor = 0
    for t in solid:
        n = len(t["col"])
        t["keys"] = np.array([key_of_label[l] for l in labels[cursor:cursor + n]], dtype=object)
        cursor += n

    # One USD file per part (UE collapses a shared root into a single static mesh).
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    for out_name, tris_list in parts.items():
        for t in tris_list:
            if t["glass"]:
                t["keys"] = np.full(len(t["col"]), GLASS_KEY, dtype=object)
        pts = np.concatenate([t["pts"] for t in tris_list])
        nrm = np.concatenate([t["nrm"] for t in tris_list])
        uv = np.concatenate([t["uv"] for t in tris_list])
        keys = np.concatenate([t["keys"] for t in tris_list])
        used = {k: materials[k] for k in sorted(set(keys.tolist()))}
        write_usd(OUT / f"ForkliftB_{out_name}.baked.usd", mpu, {out_name: (pts, nrm, uv, keys)}, used)
        (OUT / f"ForkliftB_{out_name}.materials.json").write_text(json.dumps(used, indent=2))
        flat = pts.reshape(-1, 3)
        print(f"  part {out_name:10s} {len(keys):7d} tris  bbox x[{flat[:,0].min():.2f},{flat[:,0].max():.2f}] y[{flat[:,1].min():.2f},{flat[:,1].max():.2f}] z[{flat[:,2].min():.2f},{flat[:,2].max():.2f}]")
    print("baked ForkliftB")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    # `python bake_usd.py forklift_b` bakes only that group (the C + warehouse bake takes a few minutes).
    only = set(sys.argv[1:])
    if not only or "warehouse" in only:
        for usd_name, out_name in WAREHOUSE_ASSETS:
            bake_warehouse(usd_name, out_name)
    if not only or "forklift_c" in only:
        bake_forklift()
    if not only or "forklift_b" in only:
        bake_forklift_b()
    return 0


if __name__ == "__main__":
    sys.exit(main())
