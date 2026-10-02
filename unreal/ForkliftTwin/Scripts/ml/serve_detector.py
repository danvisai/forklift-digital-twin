"""Tiny local detection server: the game POSTs a camera frame, this answers with bounding boxes.

    %USERPROFILE%\\.venvs\\forklift-ml\\Scripts\\python.exe Scripts/ml/serve_detector.py --weights Saved/ml/runs/pallet_demo_v2/weights/best.pt

Endpoints (localhost only):
    GET  /health  -> {"ok": true, "names": {...}}
    POST /detect  -> {"ms": 12.3, "width": 640, "height": 480, "boxes": [{"cls": 0, "name": "pallet", "conf": 0.91,
                                                                           "x1": ..., "y1": ..., "x2": ..., "y2": ...}]}

The body is either an encoded image (PNG/JPEG, any Content-Type starting with "image/") or raw pixels
(Content-Type application/octet-stream with X-Width, X-Height and X-Format = BGRA | RGB), which is what the game
sends so it does not have to compress every frame. Box coordinates are pixels in the frame that was sent.

Standard library HTTP only, so the only dependencies are ultralytics and pillow.
"""
import argparse
import io
import json
import pathlib
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parents[2]
LOCK = threading.Lock()  # one inference at a time; the GPU is the bottleneck anyway


def decode(handler: BaseHTTPRequestHandler, body: bytes) -> np.ndarray:
    """Return an HxWx3 BGR uint8 array - ultralytics' ndarray convention (same as cv2.imread; it
    silently assumes BGR for a raw array input, unlike a file path, which it loads and converts
    itself). Feeding it RGB here previously channel-swapped every live frame, which cost little on
    shape-robust classes like tote but was enough to keep pallet/pallet_pocket confidence under the
    serving threshold almost always - confirmed by comparing model.predict() on the same frame in
    both channel orders (BGR: ~0.7 conf, RGB: <0.1 conf)."""
    ctype = handler.headers.get("Content-Type", "")
    if ctype.startswith("image/"):
        return np.asarray(Image.open(io.BytesIO(body)).convert("RGB"))[:, :, ::-1]
    w = int(handler.headers["X-Width"])
    h = int(handler.headers["X-Height"])
    fmt = handler.headers.get("X-Format", "BGRA").upper()
    if fmt == "BGRA":
        return np.frombuffer(body, np.uint8).reshape(h, w, 4)[:, :, :3]  # already B,G,R - just drop alpha
    if fmt == "RGB":
        return np.frombuffer(body, np.uint8).reshape(h, w, 3)[:, :, ::-1]
    raise ValueError(f"unsupported X-Format {fmt}")


def make_handler(model, conf: float, imgsz: int, device):
    names = model.names

    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code: int, payload: dict) -> None:
            data = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/health":
                self._reply(200, {"ok": True, "names": names})
            else:
                self._reply(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/detect":
                self._reply(404, {"error": "not found"})
                return
            try:
                body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
                img = decode(self, body)
                t0 = time.perf_counter()
                with LOCK:
                    res = model.predict(img, conf=conf, imgsz=imgsz, device=device, verbose=False)[0]
                boxes = []
                for b in res.boxes:
                    x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                    c = int(b.cls[0])
                    boxes.append({"cls": c, "name": names[c], "conf": round(float(b.conf[0]), 4),
                                  "x1": round(x1, 1), "y1": round(y1, 1), "x2": round(x2, 1), "y2": round(y2, 1)})
                self._reply(200, {"ms": round((time.perf_counter() - t0) * 1000, 1),
                                  "width": img.shape[1], "height": img.shape[0], "boxes": boxes})
            except Exception as exc:  # report to the client instead of killing the server thread
                self._reply(400, {"error": str(exc)})

        def log_message(self, fmt, *args):  # quiet: the game calls this several times a second
            pass

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--weights", default=str(ROOT / "Saved/ml/runs/pallet_demo_v2/weights/best.pt"))
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--conf", type=float, default=0.35, help="confidence threshold")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.weights)
    # Warm up so the first real frame is not slow (CUDA init, cuDNN autotune).
    model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=args.imgsz, device=args.device, verbose=False)
    print(f"serving {args.weights} on http://127.0.0.1:{args.port}  classes={model.names}")
    ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(model, args.conf, args.imgsz, args.device)).serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
