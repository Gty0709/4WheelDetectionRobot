#!/usr/bin/env python3
"""Train YOLO26m baseline on sim_dataset; save best weights to simmodel, plots to simresult."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def _find_pkg_root() -> Path:
    for candidate in (SCRIPT_DIR.parent, SCRIPT_DIR.parents[1]):
        if (candidate / "yolo26m.pt").exists() or (candidate / "package.xml").name == "package.xml" and (candidate / "package.xml").exists():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("detection_pkg"))
    except Exception:
        return SCRIPT_DIR.parent


PKG_ROOT = _find_pkg_root()
WS_ROOT = PKG_ROOT.parents[1] if (PKG_ROOT / "package.xml").exists() else PKG_ROOT.parents[2]
sys.path.insert(0, str(SCRIPT_DIR))

import yaml
from ultralytics import YOLO

DEFAULT_DATA = WS_ROOT / "sim_dataset" / "data.yaml"
SIMMODEL_DIR = PKG_ROOT / "simmodel"
SIMRESULT_DIR = PKG_ROOT / "simresult"


def parse_args():
    p = argparse.ArgumentParser(description="Train YOLO26m on sim_dataset")
    p.add_argument("--data", type=str, default=str(DEFAULT_DATA))
    p.add_argument("--epochs", type=int, default=None)
    p.add_argument("--batch", type=int, default=None, help="Auto-probe if omitted")
    p.add_argument("--imgsz", type=int, default=None)
    p.add_argument("--device", type=str, default=None, help="RTX 4080 SUPER only (device 0)")
    p.add_argument("--workers", type=int, default=None)
    p.add_argument("--patience", type=int, default=0)
    p.add_argument("--lr0", type=float, default=0.001)
    p.add_argument("--mosaic", type=float, default=0.5)
    p.add_argument("--mixup", type=float, default=0.0)
    p.add_argument("--close-mosaic", type=int, default=10)
    p.add_argument("--name", type=str, default="train")
    return p.parse_args()


def copy_artifacts(save_dir: Path, result_dir: Path, model_dir: Path) -> None:
    result_dir.mkdir(parents=True, exist_ok=True)
    model_dir.mkdir(parents=True, exist_ok=True)

    artifact_names = {
        "results.csv",
        "results.png",
        "confusion_matrix.png",
        "confusion_matrix_normalized.png",
        "BoxPR_curve.png",
        "BoxF1_curve.png",
        "BoxP_curve.png",
        "BoxR_curve.png",
        "args.yaml",
        "labels.jpg",
        "labels_correlogram.jpg",
        "train_batch0.jpg",
        "train_batch1.jpg",
        "train_batch2.jpg",
        "val_batch0_labels.jpg",
        "val_batch0_pred.jpg",
        "val_batch1_labels.jpg",
        "val_batch1_pred.jpg",
        "val_batch2_labels.jpg",
        "val_batch2_pred.jpg",
    }
    for name in artifact_names:
        src = save_dir / name
        if src.exists():
            shutil.copy2(src, result_dir / name)

    plots_dir = result_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    for src in save_dir.glob("*.png"):
        if src.name not in artifact_names:
            shutil.copy2(src, plots_dir / src.name)

    weights = save_dir / "weights"
    if weights.exists():
        for weight in weights.glob("*.pt"):
            shutil.copy2(weight, model_dir / weight.name)


def probe_batch_size(model_path: str, data: str, imgsz: int, device: str) -> int:
    from probe_batch_size import find_max_batch

    return find_max_batch(model_path, data, imgsz, device, target_util=0.95)


def main():
    args = parse_args()
    train_cfg = yaml.safe_load((PKG_ROOT / "configs" / "train.yaml").read_text(encoding="utf-8"))
    sim_cfg = train_cfg.get("sim", {})
    baseline_cfg = train_cfg.get("baseline", {})

    epochs = args.epochs if args.epochs is not None else sim_cfg.get("epochs", 150)
    imgsz = args.imgsz if args.imgsz is not None else sim_cfg.get("imgsz", 512)
    device = args.device if args.device is not None else sim_cfg.get("device", baseline_cfg.get("device", "0"))
    workers = args.workers if args.workers is not None else sim_cfg.get("workers", 8)
    patience = args.patience if args.patience is not None else sim_cfg.get("patience", 0)

    model_rel = baseline_cfg["model"]
    model_path = str(PKG_ROOT / model_rel)
    if not Path(model_path).exists():
        model_path = str(PKG_ROOT.parent / model_rel) if (PKG_ROOT.parent / model_rel).exists() else model_path

    batch = args.batch
    if batch is None:
        batch = sim_cfg.get("batch")
    if batch is None:
        print(f"Auto-probing batch size on GPU {device} (4080 SUPER) for ~95% VRAM...")
        batch = probe_batch_size(model_path, args.data, imgsz, device)
        print(f"Selected batch size: {batch}")

    run_tmp = PKG_ROOT / "runs" / "sim_train"
    run_tmp.mkdir(parents=True, exist_ok=True)

    config_snapshot = {
        "task": "yolo26m_sim_baseline",
        "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
        "args": vars(args)
        | {"batch": batch, "epochs": epochs, "imgsz": imgsz, "device": device, "workers": workers},
        "data": args.data,
        "simmodel_dir": str(SIMMODEL_DIR),
        "simresult_dir": str(SIMRESULT_DIR),
    }
    SIMRESULT_DIR.mkdir(parents=True, exist_ok=True)
    (SIMRESULT_DIR / "config.json").write_text(
        json.dumps(config_snapshot, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    model = YOLO(model_path)
    results = model.train(
        data=args.data,
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        workers=workers,
        patience=patience,
        lr0=args.lr0,
        mosaic=args.mosaic,
        mixup=args.mixup,
        close_mosaic=args.close_mosaic,
        project=str(run_tmp),
        name=args.name,
        exist_ok=True,
        plots=True,
        verbose=True,
    )

    save_dir = Path(results.save_dir) if hasattr(results, "save_dir") else run_tmp / args.name
    copy_artifacts(save_dir, SIMRESULT_DIR, SIMMODEL_DIR)
    print(f"Training done. Best weights: {SIMMODEL_DIR / 'best.pt'}")
    print(f"Training artifacts: {SIMRESULT_DIR}")


if __name__ == "__main__":
    main()
