"""Download NVIDIA's pallet and loaded-pallet USDs into ExternalAssets/pallets/ (not committed: licence + size).

    python Scripts/download_pallets.py

Lists the public bucket the Kit prototype already uses and fetches every .usd under
Shipping/Pallets (65 files) and Shipping/Cardboard_Boxes_on_Pallet (12 files) into ExternalAssets/pallets, and
Storage/Totes (56) and Storage/Bins (14) into ExternalAssets/totes; about 460 MB.
"""
import pathlib
import re
import sys
import time
import urllib.request

BUCKET = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
ROOT = "Assets/DigitalTwin/Assets/Warehouse/"
# (bucket folder, local sub-directory under ExternalAssets)
FOLDERS = (
    ("Shipping/Pallets/", "pallets"),
    ("Shipping/Cardboard_Boxes_on_Pallet/", "pallets"),
    ("Storage/Totes/", "totes"),
    ("Storage/Bins/", "totes"),
)


def list_keys(prefix):
    keys, token = [], None
    while True:
        url = f"{BUCKET}/?list-type=2&prefix={prefix}&max-keys=1000" + (f"&continuation-token={token}" if token else "")
        xml = urllib.request.urlopen(url, timeout=60).read().decode()
        keys += re.findall(r"<Key>([^<]+)</Key>", xml)
        m = re.search(r"<NextContinuationToken>([^<]+)</NextContinuationToken>", xml)
        if not m:
            return keys
        token = m.group(1)


def main():
    base = pathlib.Path(__file__).resolve().parent.parent / "ExternalAssets"
    out = base
    for folder, sub in FOLDERS:
        out = base / sub
        out.mkdir(parents=True, exist_ok=True)
        for key in list_keys(ROOT + folder):
            if not key.endswith(".usd") or "/Materials/" in key or "Textures" in key or "Variants" in key:
                continue
            dest = out / key.split("/")[-1]
            if dest.exists() and dest.stat().st_size > 0:
                print(f"have  {dest.name}")
                continue
            print(f"fetch {dest.name}")
            for attempt in range(4):  # the bucket occasionally drops a TLS connection mid-run
                try:
                    urllib.request.urlretrieve(f"{BUCKET}/{key}", dest)
                    break
                except OSError as exc:
                    if attempt == 3:
                        raise
                    print(f"  retry ({exc})")
                    time.sleep(2 * (attempt + 1))
    print(f"done -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
