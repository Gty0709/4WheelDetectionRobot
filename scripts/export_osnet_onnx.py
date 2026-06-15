#!/usr/bin/env python3
"""Export OSNet-AIN x0_25 re-identification model to ONNX for TensorRT."""

import sys
from pathlib import Path

import torch
import torch.nn as nn


class OSNetAINx025(nn.Module):
    """Minimal wrapper to export torchreid OSNet feature extractor to ONNX."""

    def __init__(self):
        super().__init__()
        try:
            from torchreid.models import osnet_ain_x0_25
            self.backbone = osnet_ain_x0_25(num_classes=1000, pretrained=True)
        except ImportError:
            raise ImportError("torchreid not installed. Run: pip install torchreid")

    def forward(self, x):
        return self.backbone(x)


def export_osnet_onnx(output_path: str, opset: int = 17) -> Path:
    """Export OSNet-AIN x0_25 to ONNX."""
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    model = OSNetAINx025()
    model.eval()

    dummy = torch.randn(1, 3, 256, 128)
    torch.onnx.export(
        model,
        dummy,
        str(out),
        opset_version=opset,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}},
    )
    print(f"OSNet ONNX exported: {out}")
    return out


def main() -> None:
    ws_root = Path(__file__).resolve().parents[1]
    output = ws_root / "src" / "detection_pkg" / "simmodel" / "osnet_ain_x0_25.onnx"
    if output.is_file():
        print(f"Already exists: {output}")
        return
    export_osnet_onnx(str(output))


if __name__ == "__main__":
    main()
