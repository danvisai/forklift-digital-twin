"""Download the NVIDIA USD source assets used by the Kit prototype into ExternalAssets/.

These are the same files the Kit extension references over HTTPS. They are not committed (size and
NVIDIA's asset license), so run this once on a fresh checkout, then run import_assets.py inside UE.

    python Scripts/download_assets.py
"""
import pathlib
import sys
import urllib.request

WAREHOUSE = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse"
ISAAC = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots"
# The shell/lighting kit (walls, floor, ceiling, roof beam, pendant light) is the actual modular kit
# NVIDIA's own Isaac Sim "Simple_Warehouse" training scene (warehouse.usd / warehouse_multiple_shelves.usd)
# is built from - same S3 bucket, different catalog. Pulling the real tiles (and their true 9 m wall
# height: 3 stacked 3 m tiers) is what makes this look like that scene instead of flat colour boxes.
ISAAC_ENV = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Environments/Simple_Warehouse"

# local name -> URL. Keep in sync with the constants at the top of extension.py.
ASSETS = {
    "forklift_c.usd": f"{ISAAC}/Forklift/forklift_c.usd",
    "forklift_b.usd": f"{ISAAC}/Forklift/forklift_b.usd",
    "Rack_A01_PR_NVD_01.usd": f"{WAREHOUSE}/Shelving/Racks/Rack_A/Rack_A01_PR_NVD_01.usd",
    "Pallet_Asm_A04_120x122x75cm_PR_V_NVD_01.usd": f"{WAREHOUSE}/Shipping/Cardboard_Boxes_on_Pallet/Pallet_Asm_A/Pallet_Asm_A04_120x122x75cm_PR_V_NVD_01.usd",
    "BlockPallet_A01_PR_NVD_01.usd": f"{WAREHOUSE}/Shipping/Pallets/Wood/Block_A/BlockPallet_A01_PR_NVD_01.usd",
    "Bin_A06_30x51x19cm_PR_V_NVD_01.usd": f"{WAREHOUSE}/Storage/Bins/Bin_A/Bin_A06_30x51x19cm_PR_V_NVD_01.usd",
    "GuardRail_A01_01.usd": f"{WAREHOUSE}/Safety/Barriers/GuardRail_A/GuardRail_A01_01.usd",
    "SM_WallA_6M.usd": f"{ISAAC_ENV}/Props/SM_WallA_6M.usd",
    "SM_CeilingA_6X6.usd": f"{ISAAC_ENV}/Props/SM_CeilingA_6X6.usd",
    "SM_floor02.usd": f"{ISAAC_ENV}/Props/SM_floor02.usd",
    "SM_LampCeilingA_04.usd": f"{ISAAC_ENV}/Props/SM_LampCeilingA_04.usd",
    "SM_BeamA_9M.usd": f"{ISAAC_ENV}/Props/SM_BeamA_9M.usd",
    # Real multi-tier pallet racking (grey frame + orange beams) and the props that stock it, from the
    # same Simple_Warehouse scene - what warehouse_multiple_shelves.usd actually builds its racks from,
    # in place of the single-bay DigitalTwin Rack_A01 (a different, blue-painted asset).
    "SM_RackFrame_03.usd": f"{ISAAC_ENV}/Props/SM_RackFrame_03.usd",
    "SM_RackShelf_01.usd": f"{ISAAC_ENV}/Props/SM_RackShelf_01.usd",
    "SM_Rackshield_02.usd": f"{ISAAC_ENV}/Props/SM_Rackshield_02.usd",
    "SM_PaletteA_01.usd": f"{ISAAC_ENV}/Props/SM_PaletteA_01.usd",
    "SM_CardBoxA_01.usd": f"{ISAAC_ENV}/Props/SM_CardBoxA_01.usd",
    "SM_CardBoxA_02.usd": f"{ISAAC_ENV}/Props/SM_CardBoxA_02.usd",
}

# D (albedo) + ORM tiles for the shell/lighting kit, sampled in bake_usd.py the same way the forklift's
# atlas is sampled (average colour / roughness / metallic - UE gets simple flat materials, not the MDL
# textures directly).
SHELL_TEXTURE_BASE = f"{ISAAC_ENV}/Materials/Textures"
SHELL_TEXTURES = {
    f"T_{name}.png": f"{SHELL_TEXTURE_BASE}/T_{name}.png"
    for name in (
        "WallA_01_D", "WallA_01_ORM",
        "WallA_02_D", "WallA_02_ORM",
        "Floor_01_D", "Floor_01_ORM",
        "BeamsA_D", "BeamsA_ORM",
        "LampCeilingA_D", "LampCeilingA_ORM",
        "RackSetA_01_D", "RackSetA_01_ORM",
        "RackSetA_02_D", "RackSetA_02_ORM",
        "RackSetA_03_D", "RackSetA_03_ORM",
        "RackSetA_04_D", "RackSetA_04_ORM",
        "RackShield_D", "RackShield_ORM",
        "PaletteA_01_D", "PaletteA_01_ORM",
        "CardBoxA_D", "CardBoxA_ORM",
    )
}

# UDIM albedo + ORM tiles of the forklift's texture atlas. The bake step samples these for per-part
# colour / roughness / metallic (UE can't consume the MDL materials or UDIM atlas directly).
TEXTURE_BASE = f"{ISAAC}/Forklift/Props/Source/Forklift_C/Textures"
TEXTURES = {
    f"T_Forklift_C01_{kind}.{tile}.png": f"{TEXTURE_BASE}/T_Forklift_C01_{kind}.{tile}.png"
    for kind in ("Albedo", "ORM")
    for tile in (1001, 1002, 1003)
}
# ForkliftB's atlas is 8 UDIM tiles and lives one directory up from the C textures.
TEXTURES.update({
    f"T_Forklift_B1_{kind}.{tile}.png": f"{ISAAC}/Forklift/Props/Source/Textures/T_Forklift_B1_{kind}.{tile}.png"
    for kind in ("Albedo", "ORM")
    for tile in range(1001, 1009)
})


def fetch(items: dict, out_dir: pathlib.Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, url in items.items():
        dest = out_dir / name
        if dest.exists() and dest.stat().st_size > 0:
            print(f"have  {name}")
            continue
        print(f"fetch {name}")
        urllib.request.urlretrieve(url, dest)


def main() -> int:
    root = pathlib.Path(__file__).resolve().parent.parent / "ExternalAssets"
    fetch(ASSETS, root)
    fetch(TEXTURES, root / "textures")
    fetch(SHELL_TEXTURES, root / "textures")
    print(f"done -> {root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
