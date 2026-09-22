"""Bake the NVIDIA pallet / loaded-pallet USDs into UE-friendly USDs and find their fork pockets.

Run with system Python (needs usd-core numpy scipy pillow matplotlib), after Scripts/download_pallets.py:

    python Scripts/bake_pallets.py                # everything
    python Scripts/bake_pallets.py BlockPallet_A01  # only assets whose name contains this

Per asset it writes, to ExternalAssets/pallets_baked/:
    <Name>.baked.usd / <Name>.materials.json     one mesh, pivot at the bottom centre of the pallet, material slots
    debug/<Name>.png                             pocket overlays (free-depth maps with the found pockets)
and one Content-side catalogue for the generator: ExternalAssets/pallets_baked/catalog.json
(import_pallets.py copies it to Content/Pallets/catalog.json).

Pockets ("fork openings") are found from geometry, not by hand: for each of the four horizontal entry
directions the pallet base is rasterised into a (lateral, height) depth map - how far a fork could travel
before hitting something - and the free openings that are closed on top by a deck and on both sides by
stringers/blocks are the pockets. Faces with the expected two pockets are kept; anything else is reported
in `excluded` in the catalogue with the reason, never guessed.

Frames: internal maths is USD (right-handed, Z up, cm). The catalogue is written in UE's frame
(cm, left-handed: Y negated) because that is what the C++ generator consumes - UE's USD importer flips Y.
"""
import json
import pathlib
import re
import sys

import numpy as np
from pxr import Usd, UsdGeom
from scipy import ndimage

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import bake_usd as bu  # read-only reuse of flatten_mesh / write_usd

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "ExternalAssets" / "pallets"
OUT = ROOT / "ExternalAssets" / "pallets_baked"
DEBUG = OUT / "debug"

# ---- Materials: name in the USD -> approximate PBR (sRGB base colour; converted to linear below) -------
MATERIALS = {
    "Wood_New_A": dict(color=(0.62, 0.45, 0.27), roughness=0.75, metallic=0.0),
    "Wood_Recycled_A": dict(color=(0.50, 0.38, 0.25), roughness=0.85, metallic=0.0),
    "M_BlockPallet_B02": dict(color=(0.58, 0.42, 0.26), roughness=0.8, metallic=0.0),
    "M_BlockPallet_B03": dict(color=(0.58, 0.42, 0.26), roughness=0.8, metallic=0.0),
    "Metal_Glossy_A": dict(color=(0.62, 0.63, 0.65), roughness=0.30, metallic=1.0),
    "Aluminium_Brushed_A": dict(color=(0.75, 0.76, 0.78), roughness=0.35, metallic=1.0),
    "Plastic_Rough_Black_A": dict(color=(0.07, 0.07, 0.075), roughness=0.70, metallic=0.0),
    "Plastic_Yellow_A": dict(color=(0.90, 0.68, 0.05), roughness=0.55, metallic=0.0),
    "Plastic_Blue_A": dict(color=(0.06, 0.22, 0.62), roughness=0.55, metallic=0.0),
    "Plastic_Grey": dict(color=(0.40, 0.41, 0.43), roughness=0.60, metallic=0.0),
    "Plastic_Tote": dict(color=(0.30, 0.34, 0.40), roughness=0.55, metallic=0.0),
    "Plastic_Black_B": dict(color=(0.05, 0.05, 0.055), roughness=0.60, metallic=0.0),
    "Plastic_White_A": dict(color=(0.85, 0.85, 0.83), roughness=0.50, metallic=0.0),
    "Pallet_Assembly_Tile": dict(color=(0.66, 0.52, 0.34), roughness=0.90, metallic=0.0),
    "Pallet_Assembly_Trim": dict(color=(0.78, 0.78, 0.74), roughness=0.60, metallic=0.0),
    "Pallet_Assembly_C_Seam": dict(color=(0.80, 0.80, 0.75), roughness=0.60, metallic=0.0),
    "Wrap": dict(color=(0.92, 0.96, 1.0), roughness=0.15, metallic=0.0, opacity=0.12),
}
FALLBACK = "Wood_New_A"


def material_for_missing(asset, mesh_name):
    """Meshes / subsets with no binding in the USD (the material library isn't shipped with the mesh)."""
    n = (asset + " " + mesh_name).lower()
    if asset.startswith(("Tote", "Bin")):
        return "Plastic_Tote"
    if "covering" in n:
        return "Wrap"
    if "aluminum" in n or "aluminium" in n:
        return "Aluminium_Brushed_A"
    if "galvanized" in n or "steel" in n:
        return "Metal_Glossy_A"
    if "plastic" in n:
        return "Plastic_Grey"
    return FALLBACK


def srgb_to_linear(c):
    c = np.asarray(c, dtype=np.float64)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def material_json(keys):
    out = {}
    for k in sorted(set(keys)):
        base = MATERIALS.get(k)
        if base is None:
            k2 = "Wrap" if "Covering" in k else FALLBACK
            print(f"    ! no material entry for '{k}', using {k2}")
            base = MATERIALS[k2]
        props = dict(base)
        props["color"] = [float(x) for x in srgb_to_linear(base["color"])]
        out[k] = props
    return out


# ---- Standards -----------------------------------------------------------------------------------------
def standard_tag(a, b):
    """a >= b, metres. 1200x1000 also absorbs the US 48x40 in (1219 x 1016)."""
    def near(x, y, t=0.045):
        return abs(a - x) < t and abs(b - y) < t
    if near(1.2, 1.0) or near(1.219, 1.016):
        return "1200x1000"
    if near(1.2, 0.8):
        return "1200x800"
    if near(1.2, 1.2) or near(1.219, 1.219):
        return "1200x1200"
    if near(1.1, 1.1) or near(1.14, 1.14):
        return "1100x1100"
    return "other"


# ---- Mesh gathering --------------------------------------------------------------------------------------
LOAD_TOKENS = ("boxes", "covering", "trim", "crate", "load")


def gather(usd_path, asset):
    """Return (tris[T,3,3] cm, normals, uvs, keys[T], is_base[T]) with the stage's own frame."""
    stage = Usd.Stage.Open(str(usd_path))
    xf = UsdGeom.XformCache()
    loaded = asset.startswith("Pallet_Asm")
    tri_l, nrm_l, uv_l, key_l, base_l = [], [], [], [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(prim).ComputePurpose() not in ("default", "render"):
            continue
        mesh = UsdGeom.Mesh(prim)
        n_faces = len(mesh.GetFaceVertexCountsAttr().Get())
        pts, nrm, uv, face_of_tri = bu.flatten_mesh(prim, xf, None)

        default = None
        rel = prim.GetRelationship("material:binding")
        if rel and rel.GetTargets():
            default = rel.GetTargets()[0].name
        if default is None:
            default = material_for_missing(asset, prim.GetName())
        face_keys = np.full(n_faces, default, dtype=object)
        for sub in UsdGeom.Subset.GetAllGeomSubsets(mesh):
            r = sub.GetPrim().GetRelationship("material:binding")
            name = r.GetTargets()[0].name if r and r.GetTargets() else material_for_missing(asset, prim.GetName())
            face_keys[np.array(sub.GetIndicesAttr().Get(), dtype=np.int64)] = name

        is_base = not (loaded and any(t in prim.GetName().lower() for t in LOAD_TOKENS))
        tri_l.append(pts)
        nrm_l.append(nrm)
        uv_l.append(uv)
        key_l.append(face_keys[face_of_tri])
        base_l.append(np.full(len(pts), is_base))
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    scale = mpu * 100.0  # to cm
    return (np.concatenate(tri_l) * scale, np.concatenate(nrm_l), np.concatenate(uv_l),
            np.concatenate(key_l), np.concatenate(base_l), mpu)


# ---- Pocket analysis ---------------------------------------------------------------------------------------
CELL = 0.5            # cm, raster cell
SAMPLE_CM2 = 0.20     # one surface sample per this much area
MIN_DEPTH = 20.0      # cm a fork must be able to travel to count as a pocket
MIN_W, MIN_H = 8.0, 3.5
MIN_FILL = 0.85
MAX_FOOTPRINT_M = 1.5  # bigger than this is a special-purpose (drum / IBC) pallet, not what a forklift picks in a hall
MIN_TOTE_M, MIN_TOTE_H = 0.30, 0.12  # smaller bins (10 cm parts bins) are not small load carriers
TOTES = ROOT / "ExternalAssets" / "totes"


def sample_surface(tris, rng):
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = np.maximum(1, np.ceil(area / SAMPLE_CM2)).astype(np.int64)
    idx = np.repeat(np.arange(len(tris)), n)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    f = u + v > 1
    u[f], v[f] = 1 - u[f], 1 - v[f]
    return a[idx] + u[:, None] * (b[idx] - a[idx]) + v[:, None] * (c[idx] - a[idx])


ENTRIES = ("-X", "+X", "-Y", "+Y")  # the side of the pallet the fork comes in from


def analyse_entry(P, lo, hi, entry):
    """P: surface samples (cm, pivot frame). Returns (pockets, depth_map, extents) for one entry side."""
    axis = 0 if entry[1] == "X" else 1
    lat = 1 - axis
    sign = -1 if entry[0] == "-" else 1          # side the fork comes from
    length = hi[axis] - lo[axis]
    depth = (P[:, axis] - lo[axis]) if sign < 0 else (hi[axis] - P[:, axis])
    ny = int(np.ceil((hi[lat] - lo[lat]) / CELL))
    nz = int(np.ceil(hi[2] / CELL))
    yi = np.clip(((P[:, lat] - lo[lat]) / CELL).astype(int), 0, ny - 1)
    zi = np.clip((P[:, 2] / CELL).astype(int), 0, nz - 1)
    dmap = np.full((ny, nz), np.inf)
    np.minimum.at(dmap, (yi, zi), depth)
    free = dmap >= MIN_DEPTH
    labels, n = ndimage.label(free)
    pockets = []
    for k in range(1, n + 1):
        ys, zs = np.nonzero(labels == k)
        if ys.min() == 0 or ys.max() == ny - 1 or zs.max() == nz - 1:
            continue  # open to the side or to the top: not an enclosed fork opening
        w, h = (ys.max() - ys.min() + 1) * CELL, (zs.max() - zs.min() + 1) * CELL
        fill = len(ys) / ((ys.max() - ys.min() + 1) * (zs.max() - zs.min() + 1))
        if w < MIN_W or h < MIN_H or fill < MIN_FILL:
            continue
        d = np.minimum(dmap[ys, zs], length)
        y0, y1 = lo[lat] + ys.min() * CELL, lo[lat] + (ys.max() + 1) * CELL
        z0, z1 = zs.min() * CELL, (zs.max() + 1) * CELL
        pockets.append(dict(lat=(y0, y1), z=(z0, z1), depth=float(np.percentile(d, 5)), fill=float(fill)))
    pockets.sort(key=lambda p: p["lat"][0])
    return pockets, dmap, (lo[lat], hi[lat], hi[2]), (axis, lat, sign)


def pocket_box_usd(entry, pk, lo, hi):
    """Opening rectangle as a box in the pivot frame (cm, USD): centre, size (x,y,z), outward normal."""
    axis = 0 if entry[1] == "X" else 1
    sign = -1 if entry[0] == "-" else 1
    depth = pk["depth"]
    c = np.zeros(3)
    s = np.zeros(3)
    edge = lo[axis] if sign < 0 else hi[axis]
    # Inward is -sign: the box spans [edge, edge + (-sign) * depth].
    c[axis] = edge + (-sign) * depth / 2.0
    lat = 1 - axis
    c[lat] = 0.5 * (pk["lat"][0] + pk["lat"][1])
    c[2] = 0.5 * (pk["z"][0] + pk["z"][1])
    s[axis], s[lat], s[2] = depth, pk["lat"][1] - pk["lat"][0], pk["z"][1] - pk["z"][0]
    n = np.zeros(3)
    n[axis] = sign  # outward normal points to the side the fork comes from
    return c, s, n


def usd_to_ue(v):
    return [float(v[0]), float(-v[1]), float(v[2])]


# ---- Per asset -------------------------------------------------------------------------------------------
def classify(asset):
    return "loaded_pallet" if asset.startswith("Pallet_Asm") else "pallet"


def process(usd_path, rng):
    asset = usd_path.stem.replace("_PR_V_NVD_01", "").replace("_PR_NVD_01", "")
    cls = classify(asset)
    tris, nrm, uv, keys, is_base, mpu = gather(usd_path, asset)

    base = tris[is_base]
    b_lo, b_hi = base.reshape(-1, 3).min(0), base.reshape(-1, 3).max(0)
    a_lo, a_hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    # Pivot: XY centre of the pallet itself, Z at the lowest point.
    pivot = np.array([(b_lo[0] + b_hi[0]) / 2, (b_lo[1] + b_hi[1]) / 2, a_lo[2]])
    tris = tris - pivot
    b_lo, b_hi = b_lo - pivot, b_hi - pivot
    a_lo, a_hi = a_lo - pivot, a_hi - pivot

    fp = sorted([(b_hi[0] - b_lo[0]) / 100, (b_hi[1] - b_lo[1]) / 100], reverse=True)
    info = dict(
        name=asset, cls=cls, standard=standard_tag(fp[0], fp[1]),
        footprint_m=[round(fp[0], 3), round(fp[1], 3)],
        pallet_height_m=round(float(b_hi[2]) / 100, 3),
        bbox_cm_usd=dict(lo=a_lo.tolist(), hi=a_hi.tolist()),
        tris=int(len(tris)),
    )

    # Bake (units stay cm, stage mpu 0.01 like the warehouse assets).
    materials = material_json(keys)
    OUT.mkdir(parents=True, exist_ok=True)
    bu.write_usd(OUT / f"{asset}.baked.usd", 0.01, {asset: (tris, nrm, uv, keys)}, materials)
    (OUT / f"{asset}.materials.json").write_text(json.dumps(materials, indent=2))
    info["material_slots"] = sorted(materials)

    # Pockets from the pallet base only, below the top of the pallet.
    P = sample_surface(tris[is_base], rng)
    faces, dbg = {}, {}
    for entry in ENTRIES:
        pockets, dmap, ext, ax = analyse_entry(P, b_lo, b_hi, entry)
        dbg[entry] = (dmap, pockets, ext, ax)
        if pockets:
            faces[entry] = pockets
    good, problems = {}, []
    for entry, pk in faces.items():
        if len(pk) == 2 and abs((pk[0]["lat"][1] - pk[0]["lat"][0]) - (pk[1]["lat"][1] - pk[1]["lat"][0])) < 6.0:
            good[entry] = pk
        else:
            problems.append(f"{entry}: {len(pk)} pockets")
    if fp[0] > MAX_FOOTPRINT_M:
        info["excluded"] = f"oversize ({fp[0]:.2f} x {fp[1]:.2f} m): not a normal forklift pallet"
    elif not good:
        info["excluded"] = "no entry face with two matching pockets" + (f" ({'; '.join(problems)})" if problems else "")
    else:
        if problems:
            info["note"] = "ignored faces: " + "; ".join(problems)
        out = []
        for entry, pks in good.items():
            for pk in pks:
                c, s, n = pocket_box_usd(entry, pk, b_lo, b_hi)
                out.append(dict(entry_ue=(entry[0] + entry[1]) if entry[1] == "X" else (("+" if entry[0] == "-" else "-") + "Y"),
                                entry_usd=entry, center_cm=usd_to_ue(c), size_cm=[float(x) for x in s],
                                normal=usd_to_ue(n), depth_cm=round(pk["depth"], 1)))
        info["pockets"] = out
    draw_debug(asset, info, dbg, good)
    # UE-frame extents of the whole object (for the 3D box of the class label).
    info["bbox_cm"] = dict(lo=usd_to_ue([a_lo[0], a_hi[1], a_lo[2]]), hi=usd_to_ue([a_hi[0], a_lo[1], a_hi[2]]))
    info["base_bbox_cm"] = dict(lo=usd_to_ue([b_lo[0], b_hi[1], b_lo[2]]), hi=usd_to_ue([b_hi[0], b_lo[1], b_hi[2]]))
    del info["bbox_cm_usd"]
    return info


def process_prop(usd_path):
    """Totes and bins (class 'tote'): bake with the pivot at the bottom centre; no pockets."""
    asset = usd_path.stem.replace("_PR_V_NVD_01", "").replace("_PR_NVD_01", "")
    tris, nrm, uv, keys, _, _ = gather(usd_path, asset)
    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    size = (hi - lo) / 100.0
    info = dict(name=asset, cls="tote", size_m=[round(float(x), 3) for x in size])
    if max(size[0], size[1]) < MIN_TOTE_M or size[2] < MIN_TOTE_H:
        info["excluded"] = f"too small to be a small load carrier ({size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m)"
        return info
    if max(size[0], size[1]) > 0.9:
        info["excluded"] = f"too large for a tote ({size[0]:.2f} x {size[1]:.2f} m)"
        return info
    pivot = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    tris = tris - pivot
    a_lo, a_hi = lo - pivot, hi - pivot
    materials = material_json(keys)
    OUT.mkdir(parents=True, exist_ok=True)
    bu.write_usd(OUT / f"{asset}.baked.usd", 0.01, {asset: (tris, nrm, uv, keys)}, materials)
    (OUT / f"{asset}.materials.json").write_text(json.dumps(materials, indent=2))
    info["material_slots"] = sorted(materials)
    info["tris"] = int(len(tris))
    info["bbox_cm"] = dict(lo=usd_to_ue([a_lo[0], a_hi[1], a_lo[2]]), hi=usd_to_ue([a_hi[0], a_lo[1], a_hi[2]]))
    return info


def draw_debug(asset, info, dbg, good):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    DEBUG.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 4, figsize=(16, 2.6))
    for ax, entry in zip(axs, ENTRIES):
        dmap, pockets, (lat_lo, lat_hi, zmax), _ = dbg[entry]
        img = np.where(np.isinf(dmap), 40, np.minimum(dmap, 40)).T  # (z, lat); cm of free travel, capped
        ax.imshow(img, origin="lower", cmap="viridis", extent=[lat_lo, lat_hi, 0, zmax], aspect="equal")
        for pk in pockets:
            ax.add_patch(Rectangle((pk["lat"][0], pk["z"][0]), pk["lat"][1] - pk["lat"][0], pk["z"][1] - pk["z"][0],
                                   fill=False, ec="red" if entry in good else "orange", lw=1.5))
        ax.set_title(f"enter from {entry}: {len(pockets)} pockets" + ("" if entry in good else " (dropped)"), fontsize=8)
        ax.tick_params(labelsize=6)
    fig.suptitle(f"{asset}  [{info['cls']}, {info['standard']}, {info['footprint_m']}, h={info['pallet_height_m']}]"
                 + ("  EXCLUDED" if "excluded" in info else ""), fontsize=9)
    fig.tight_layout()
    fig.savefig(DEBUG / f"{asset}.png", dpi=60)
    plt.close(fig)


def main():
    want = sys.argv[1] if len(sys.argv) > 1 else ""
    rng = np.random.default_rng(1234)
    catalog = dict(frame="UE (cm, left-handed, pivot = bottom centre of the pallet, +X/+Y as the mesh imports)", assets=[], excluded=[])
    for usd_path in sorted(SRC.glob("*.usd")):
        if want and want not in usd_path.name:
            continue
        print(f"== {usd_path.name}")
        try:
            info = process(usd_path, rng)
        except Exception as exc:  # keep going: one odd asset must not stop the batch
            print(f"   ! FAILED {exc!r}")
            catalog["excluded"].append(dict(name=usd_path.stem, reason=f"bake failed: {exc!r}"))
            continue
        tag = f"{info['cls']} {info['standard']} {info['footprint_m']} h={info['pallet_height_m']}"
        if "excluded" in info:
            print(f"   EXCLUDED: {info['excluded']}   [{tag}]")
            catalog["excluded"].append(dict(name=info["name"], reason=info["excluded"], standard=info["standard"], cls=info["cls"]))
        else:
            print(f"   ok: {len(info['pockets'])} pockets on {sorted({p['entry_usd'] for p in info['pockets']})}   [{tag}]" + (f"  note: {info['note']}" if "note" in info else ""))
            catalog["assets"].append(info)
    catalog["props"] = []
    for usd_path in sorted(TOTES.glob("*.usd")):
        if want and want not in usd_path.name:
            continue
        try:
            info = process_prop(usd_path)
        except Exception as exc:
            print(f"   ! {usd_path.name} FAILED {exc!r}")
            catalog["excluded"].append(dict(name=usd_path.stem, reason=f"bake failed: {exc!r}"))
            continue
        if "excluded" in info:
            print(f"   {info['name']}: EXCLUDED {info['excluded']}")
            catalog["excluded"].append(dict(name=info["name"], reason=info["excluded"], cls="tote"))
        else:
            print(f"   {info['name']}: tote {info['size_m']}")
            catalog["props"].append(info)
    if not want:
        (OUT / "catalog.json").write_text(json.dumps(catalog, indent=1))
        print(f"catalog: {len(catalog['assets'])} usable pallets, {len(catalog['props'])} totes/bins, {len(catalog['excluded'])} excluded -> {OUT / 'catalog.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
