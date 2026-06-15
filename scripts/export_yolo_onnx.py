#!/usr/bin/env python3
"""Export YOLO models to ONNX format for TensorRT conversion."""

import sys
from pathlib import Path


def export_yolo_onnx(weights_path: str, output_path: str | None = None, imgsz: int = 640) -> Path:
    """Export a YOLO .pt model to ONNX format."""
    from ultralytics import YOLO

    weights = Path(weights_path)
    if not weights.is_file():
        raise FileNotFoundError(f"Weights not found: {weights}")

    model = YOLO(str(weights))
    out = model.export(format="onnx", imgsz=imgsz, simplify=True, opset=17)
    result = Path(out)
    if output_path and Path(output_path) != result:
        target = Path(output_path)
        target.parent.mkdir(parents=True, exist_ok=True)
        result.rename(target)
        return target
    return result


def main() -> None:
    ws_root = Path(__file__).resolve().parents[1]
    simmodel_dir = ws_root / "src" / "detection_pkg" / "simmodel"

    models = [
        (simmodel_dir / "best.pt", simmodel_dir / "best.onnx"),
        (ws_root / "yolo26n.pt", ws_root / "src" / "detection_pkg" / "yolo26n.onnx"),
    ]

    for weights, onnx_out in models:
        if not weights.is_file():
            print(f"Skipping {weights} (not found)")
            continue
        if onnx_out.is_file():
            print(f"Already exists: {onnx_out}")
            continue
        print(f"Exporting {weights} -> {onnx_out} ...")
        result = export_yolo_onnx(str(weights), str(onnx_out))
        print(f"  Done: {result}")


if __name__ == "__main__":
    main()
