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

# local name -> URL. Keep in sync with the constants at the top of extension.py.
ASSETS = {
    "forklift_c.usd": f"{ISAAC}/Forklift/forklift_c.usd",
    "forklift_b.usd": f"{ISAAC}/Forklift/forklift_b.usd",
    "Rack_A01_PR_NVD_01.usd": f"{WAREHOUSE}/Shelving/Racks/Rack_A/Rack_A01_PR_NVD_01.usd",
    "Pallet_Asm_A04_120x122x75cm_PR_V_NVD_01.usd": f"{WAREHOUSE}/Shipping/Cardboard_Boxes_on_Pallet/Pallet_Asm_A/Pallet_Asm_A04_120x122x75cm_PR_V_NVD_01.usd",
    "BlockPallet_A01_PR_NVD_01.usd": f"{WAREHOUSE}/Shipping/Pallets/Wood/Block_A/BlockPallet_A01_PR_NVD_01.usd",
    "Bin_A06_30x51x19cm_PR_V_NVD_01.usd": f"{WAREHOUSE}/Storage/Bins/Bin_A/Bin_A06_30x51x19cm_PR_V_NVD_01.usd",
    "GuardRail_A01_01.usd": f"{WAREHOUSE}/Safety/Barriers/GuardRail_A/GuardRail_A01_01.usd",
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
    print(f"done -> {root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
