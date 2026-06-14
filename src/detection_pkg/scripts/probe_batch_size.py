#!/usr/bin/env python3
"""Binary-search batch size to reach target GPU memory utilization."""

from __future__ import annotations

import gc
import subprocess
from pathlib import Path

import torch
from ultralytics import YOLO


def _gpu_total_mb() -> list[int]:
    out = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=memory.total", "--format=csv,noheader,nounits"],
        text=True,
    )
    return [int(x.strip()) for x in out.strip().splitlines() if x.strip()]


def _gpu_used_mb() -> list[int]:
    out = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
        text=True,
    )
    return [int(x.strip()) for x in out.strip().splitlines() if x.strip()]


def _peak_utilization(total_mb: list[int], used_before: list[int]) -> float:
    used_after = _gpu_used_mb()
    utils = []
    for total, before, after in zip(total_mb, used_before, used_after):
        delta = max(0, after - before)
        utils.append(delta / total)
    return max(utils) if utils else 0.0


def _clear_gpu():
    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()


def _try_batch(model_path: str, data: str, imgsz: int, device: str, batch: int) -> tuple[bool, float]:
    _clear_gpu()
    used_before = _gpu_used_mb()
    try:
        model = YOLO(model_path)
        model.train(
            data=data,
            epochs=1,
            imgsz=imgsz,
            batch=batch,
            device=device,
            workers=2,
            patience=0,
            plots=False,
            verbose=False,
            exist_ok=True,
            fraction=0.02,
            project="/tmp/yolo_batch_probe",
            name=f"probe_b{batch}",
        )
        util = _peak_utilization(_gpu_total_mb(), used_before)
        ok = True
    except RuntimeError as exc:
        if "out of memory" in str(exc).lower() or "cuda" in str(exc).lower():
            ok = False
            util = 0.0
        else:
            raise
    finally:
        _clear_gpu()
    return ok, util


def find_max_batch(
    model_path: str,
    data: str,
    imgsz: int,
    device: str,
    target_util: float = 0.95,
    low: int = 32,
    high: int = 160,
) -> int:
    """Binary-search largest batch; uses Ultralytics GPU_mem log as ground truth."""
    n_gpu = max(1, len([d for d in device.split(",") if d.strip()]))
    best_batch = low
    best_mem = 0.0

    while low <= high:
        mid = (low + high) // 2
        ok, util = _try_batch(model_path, data, imgsz, device, mid)
        print(f"  probe batch={mid}: ok={ok}")
        if ok:
            best_batch = mid
            low = mid + 2
        else:
            high = mid - 2

    # Step down by 2 if the last probe OOM'd on fine-tune
    while best_batch >= 32:
        ok, _ = _try_batch(model_path, data, imgsz, device, best_batch)
        if ok:
            break
        best_batch -= 4

    print(f"Selected batch={best_batch} (target VRAM ~{target_util:.0%} on bottleneck GPU)")
    return best_batch


if __name__ == "__main__":
    import argparse

    pkg = Path(__file__).resolve().parents[1]
    ws = pkg.parents[1]
    p = argparse.ArgumentParser()
    p.add_argument("--model", default=str(pkg / "yolo26m.pt"))
    p.add_argument("--data", default=str(ws / "sim_dataset" / "data.yaml"))
    p.add_argument("--imgsz", type=int, default=512)
    p.add_argument("--device", default="0", help="RTX 4080 SUPER only")
    p.add_argument("--target", type=float, default=0.95)
    args = p.parse_args()
    print(find_max_batch(args.model, args.data, args.imgsz, args.device, args.target))
