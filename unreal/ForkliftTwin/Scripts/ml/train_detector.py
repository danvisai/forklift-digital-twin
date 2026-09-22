"""Train the pallet / loaded pallet / pocket / tote / rack detector on the synthetic dataset.

Run with the ML virtualenv (CUDA PyTorch + ultralytics), NOT the system Python:

    %USERPROFILE%\\.venvs\\forklift-ml\\Scripts\\python.exe Scripts/ml/train_detector.py --data Saved/Dataset/<name>/data.yaml

Weights land in Saved/ml/runs/<name>/weights/best.pt, which serve_detector.py loads.

The GPU on this machine is thermally throttled, so keep the model small (yolo11n) and stop UE renders while
this runs; they compete for the same GPU.

Note: ultralytics is AGPL-3.0. Fine for this prototype; revisit (RT-DETR / RF-DETR are Apache-2.0) before shipping.
"""
import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="path to the dataset's data.yaml")
    ap.add_argument("--model", default="yolo11n.pt", help="pretrained checkpoint to start from (downloaded on first use)")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--name", default="pallet_demo")
    ap.add_argument("--device", default="0", help="'0' for the GPU, 'cpu' to force CPU (very slow)")
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    import torch
    from ultralytics import YOLO

    if args.device != "cpu" and not torch.cuda.is_available():
        print("CUDA is not available in this Python (CPU-only torch?). Use the forklift-ml venv, or pass --device cpu.")
        return 1

    data = pathlib.Path(args.data).resolve()
    if not data.exists():
        print(f"data.yaml not found: {data}")
        return 1

    model = YOLO(args.model)
    model.train(
        data=str(data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(ROOT / "Saved" / "ml" / "runs"),
        name=args.name,
        exist_ok=True,
        # The scene is rendered by one renderer with fixed exposure, so heavy colour jitter mostly adds noise.
        # Horizontal flips are safe (pallets are left-right symmetric); vertical flips are not (gravity).
        fliplr=0.5,
        flipud=0.0,
        hsv_h=0.01,
        hsv_s=0.4,
        hsv_v=0.3,
        mosaic=0.5,
        patience=15,
        plots=True,
    )
    best = ROOT / "Saved" / "ml" / "runs" / args.name / "weights" / "best.pt"
    print(f"\nDone. Best weights: {best}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
