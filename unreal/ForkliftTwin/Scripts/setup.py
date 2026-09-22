"""One-shot setup: download NVIDIA sources -> bake -> build C++ -> import into UE -> build the map.

    python Scripts/setup.py                       # UE 5.8 at the default launcher location
    python Scripts/setup.py --ue "D:/UE_5.8"      # or set UE_ROOT
    python Scripts/setup.py --skip-download       # reuse ExternalAssets/

Needs: Windows, UE 5.8, Visual Studio 2022 (C++ workload), Python 3 with `pip install usd-core numpy pillow`.
Close any running Unreal Editor first (Live Coding blocks the build).
"""
import argparse
import os
import pathlib
import shutil
import subprocess
import sys

PROJECT = pathlib.Path(__file__).resolve().parent.parent
SCRIPTS = PROJECT / "Scripts"
UPROJECT = PROJECT / "ForkliftTwin.uproject"


def run(cmd, **kwargs):
    print("\n>>", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def unreal_python(ue_root: pathlib.Path, script: pathlib.Path):
    editor = ue_root / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe"
    run([editor, UPROJECT, "-unattended", "-nullrhi", "-nosplash", "-nosound", "-stdout", "-FullStdOutLogOutput",
         f"-ExecutePythonScript={script}"])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ue", default=os.environ.get("UE_ROOT", "C:/Program Files/Epic Games/UE_5.8"))
    parser.add_argument("--skip-download", action="store_true")
    parser.add_argument("--skip-bake", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()

    ue_root = pathlib.Path(args.ue)
    if not (ue_root / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe").exists():
        print(f"Unreal Engine not found at {ue_root}. Pass --ue or set UE_ROOT.")
        return 1

    if not args.skip_download:
        run([sys.executable, SCRIPTS / "download_assets.py"])
    if not args.skip_bake:
        run([sys.executable, SCRIPTS / "bake_usd.py"])
    if not args.skip_build:
        run([ue_root / "Engine/Build/BatchFiles/Build.bat", "ForkliftTwinEditor", "Win64", "Development",
             f"-Project={UPROJECT}", "-WaitMutex", "-NoHotReload"])

    # Start the import from a clean Content tree on disk: re-importing over existing assets in one editor
    # session leaves stale registry entries that break renames.
    for sub in ("Forklift", "ForkliftB", "Warehouse", "Maps", "_ImportTemp"):
        shutil.rmtree(PROJECT / "Content" / sub, ignore_errors=True)

    unreal_python(ue_root, SCRIPTS / "import_assets.py")
    unreal_python(ue_root, SCRIPTS / "build_level.py")

    print("\nDone. Open ForkliftTwin.uproject, or launch the game directly:")
    print(f'  "{ue_root}/Engine/Binaries/Win64/UnrealEditor.exe" "{UPROJECT}" /Game/Maps/Warehouse -game')
    return 0


if __name__ == "__main__":
    sys.exit(main())
