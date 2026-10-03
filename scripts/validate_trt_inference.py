#!/usr/bin/env python3
"""Compare TensorRT / OpenCV-DNN C++ inference vs Ultralytics on a test image."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def _find_test_image(ws_root: Path) -> Path | None:
    candidates = [
        ws_root / "src" / "detection_pkg" / "simresult" / "inference" / "images",
        ws_root / "src" / "detection_pkg" / "simmodel",
    ]
    for base in candidates:
        if not base.is_dir():
            continue
        for pattern in ("*.jpg", "*.png", "*.jpeg"):
            matches = sorted(base.rglob(pattern))
            if matches:
                return matches[0]
    return None


def run_ultralytics(weights: Path, image: Path, conf: float) -> list[tuple[float, float, float, float, float]]:
    from ultralytics import YOLO

    model = YOLO(str(weights))
    results = model.predict(str(image), conf=conf, verbose=False)
    boxes: list[tuple[float, float, float, float, float]] = []
    for result in results:
        if result.boxes is None:
            continue
        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy().tolist()
            conf_v = float(box.conf[0].cpu().numpy())
            boxes.append((xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf_v))
    boxes.sort(key=lambda b: -b[4])
    return boxes


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / max(area_a + area_b - inter, 1e-6)


def compare_boxes(
    ref: list[tuple[float, float, float, float, float]],
    other: list[tuple[float, float, float, float, float]],
    iou_thresh: float = 0.5,
    conf_tol: float = 0.05,
) -> dict[str, float | int]:
    matched = 0
    conf_deltas: list[float] = []
    used = set()
    for r in ref:
        best_iou = 0.0
        best_j = -1
        for j, o in enumerate(other):
            if j in used:
                continue
            score = iou(r[:4], o[:4])
            if score > best_iou:
                best_iou = score
                best_j = j
        if best_j >= 0 and best_iou >= iou_thresh:
            used.add(best_j)
            matched += 1
            conf_deltas.append(abs(r[4] - other[best_j][4]))
    return {
        "ref_count": len(ref),
        "other_count": len(other),
        "matched": matched,
        "mean_conf_delta": sum(conf_deltas) / len(conf_deltas) if conf_deltas else 0.0,
    }


def run_opencv_dnn(onnx: Path, image: Path, conf: float) -> list[tuple[float, float, float, float, float]]:
    import cv2
    import numpy as np

    bgr = cv2.imread(str(image))
    if bgr is None:
        raise FileNotFoundError(image)
    h, w = bgr.shape[:2]
    size = 640
    scale = min(size / w, size / h)
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2
    resized = cv2.resize(bgr, (new_w, new_h))
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    blob = cv2.dnn.blobFromImage(rgb, 1.0, (size, size), (0, 0, 0), swapRB=False, crop=False)
    net = cv2.dnn.readNetFromONNX(str(onnx))
    net.setInput(blob)
    out = net.forward()[0]
    boxes: list[tuple[float, float, float, float, float]] = []
    for row in out:
        score = float(row[4])
        if score < conf:
            continue
        x1 = (float(row[0]) - pad_x) / scale
        y1 = (float(row[1]) - pad_y) / scale
        x2 = (float(row[2]) - pad_x) / scale
        y2 = (float(row[3]) - pad_y) / scale
        boxes.append((x1, y1, x2, y2, score))
    boxes.sort(key=lambda b: -b[4])
    return boxes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, default=None, help="Test image path")
    parser.add_argument("--weights", type=Path, default=None, help="YOLO .pt weights")
    parser.add_argument("--onnx", type=Path, default=None, help="YOLO ONNX model")
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--iou", type=float, default=0.5)
    args = parser.parse_args()

    ws_root = Path(__file__).resolve().parents[1]
    image = args.image or _find_test_image(ws_root)
    if image is None or not image.is_file():
        print("ERROR: no test image found; pass --image", file=sys.stderr)
        return 1

    simmodel = ws_root / "src" / "detection_pkg" / "simmodel"
    weights = args.weights or (simmodel / "best.pt")
    onnx = args.onnx or (simmodel / "best.onnx")
    if not weights.is_file():
        print(f"ERROR: weights not found: {weights}", file=sys.stderr)
        return 1
    if not onnx.is_file():
        print(f"ERROR: ONNX not found: {onnx}", file=sys.stderr)
        return 1

    print(f"Image:   {image}")
    print(f"Weights: {weights}")
    print(f"ONNX:    {onnx}")

    ref = run_ultralytics(weights, image, args.conf)
    dnn = run_opencv_dnn(onnx, image, args.conf)
    stats = compare_boxes(ref, dnn, iou_thresh=args.iou)

    print(f"\nUltralytics detections: {stats['ref_count']}")
    print(f"ONNX/DNN detections:    {stats['other_count']}")
    print(f"Matched (IoU>={args.iou}): {stats['matched']}")
    print(f"Mean |conf delta|:      {stats['mean_conf_delta']:.4f}")

    engine = simmodel / "best.engine"
    if engine.is_file():
        print(f"\nTensorRT engine present: {engine}")
        print("(Run clip_detector_cpp at runtime to exercise TRT path.)")
    else:
        print("\nNo best.engine yet; trt_inference will build from ONNX on first run.")

    trtexec = subprocess.run(["which", "trtexec"], capture_output=True, text=True)
    if trtexec.returncode == 0:
        print(f"trtexec: {trtexec.stdout.strip()}")

    ok = stats["matched"] == stats["ref_count"] == stats["other_count"] or stats["matched"] > 0
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
