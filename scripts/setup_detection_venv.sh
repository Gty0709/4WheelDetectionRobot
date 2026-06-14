#!/usr/bin/env bash
# 仅为 detection 准备 NumPy 1.x + OpenCV 隔离层（~50MB），不重复安装 torch/ultralytics。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="$ROOT/.detection-python"

echo "[setup_detection_env] workspace: $ROOT"
echo "[setup_detection_env] target:    $TARGET"

mkdir -p "$TARGET"
python3 -m pip install --upgrade pip wheel
python3 -m pip install --target "$TARGET" --upgrade \
  'numpy<2' \
  'opencv-python-headless>=4.8,<4.10'

date -Iseconds > "$TARGET/.installed"

if ! python3 -c "import ultralytics" 2>/dev/null; then
  echo "[setup_detection_env] 警告: 未在默认 Python 中找到 ultralytics，训练/检测需先: pip install ultralytics" >&2
fi

if ! python3 -c "import torchreid" 2>/dev/null; then
  echo "[setup_detection_env] 可选 ReID: pip install --user torchreid  (OSNet 外观重识别，失败时自动降级 ORB)" >&2
fi

PYTHONPATH="$TARGET" python3 - <<'PY'
import numpy
import cv2
print(f"  numpy {numpy.__version__} | opencv {cv2.__version__}")
PY

echo "[setup_detection_env] 完成。全局 ~/.local 不会被修改。"
