"""Run a trained detector over a folder of images and save copies with the boxes drawn on them.

    %USERPROFILE%\\.venvs\\forklift-ml\\Scripts\\python.exe Scripts/ml/predict_folder.py --images Saved/SensorTest --weights <best.pt>

Output goes to Saved/ml/predictions/<name>/. Useful for eyeballing the model on fork-camera frames before the
live in-game overlay is wired up (that one uses serve_detector.py).
"""
import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True, help="folder (or single file) of PNG/JPG images")
    ap.add_argument("--weights", default=str(ROOT / "Saved/ml/runs/pallet_demo_v2/weights/best.pt"))
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    ap.add_argument("--name", default="demo")
    args = ap.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.weights)
    results = model.predict(
        args.images, conf=args.conf, imgsz=args.imgsz, device=args.device, save=True,
        project=str(ROOT / "Saved" / "ml" / "predictions"), name=args.name, exist_ok=True, verbose=False,
    )
    for r in results:
        found = ", ".join(f"{model.names[int(c)]} {float(s):.2f}" for c, s in zip(r.boxes.cls, r.boxes.conf)) or "nothing"
        print(f"{pathlib.Path(r.path).name}: {found}")
    print(f"annotated images -> {ROOT / 'Saved' / 'ml' / 'predictions' / args.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
