#!/usr/bin/env bash
# Export missing ONNX (OSNet) and build all TensorRT .engine files.
# Prefers trtexec when available; otherwise uses detection_pkg build_trt_engines.
set -euo pipefail

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export ROBOTHOMEWORK_WS="$WS_ROOT"
SIMMODEL="$WS_ROOT/src/detection_pkg/simmodel"

echo "=== Step 1: export OSNet ONNX (if missing) ==="
if [ ! -f "$SIMMODEL/osnet_ain_x0_25.onnx" ]; then
  python3 "$WS_ROOT/scripts/export_osnet_onnx.py"
else
  echo "Already exists: $SIMMODEL/osnet_ain_x0_25.onnx"
fi

echo "=== Step 2: verify YOLO ONNX ==="
for onnx in "$SIMMODEL/best.onnx" "$WS_ROOT/src/detection_pkg/yolo26n.onnx"; do
  if [ ! -f "$onnx" ]; then
    echo "WARN: missing $onnx (run scripts/export_yolo_onnx.py if needed)"
  fi
done

echo "=== Step 3: build TensorRT engines ==="
if command -v trtexec &>/dev/null || [ -x /usr/local/cuda/bin/trtexec ]; then
  bash "$WS_ROOT/scripts/convert_to_trt_engine.sh" "$@"
else
  echo "trtexec not found; using build_trt_engines (C++ TensorRT builder)"
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
  fi
  BUILDER="$WS_ROOT/install/detection_pkg/lib/detection_pkg/build_trt_engines"
  if [ ! -x "$BUILDER" ]; then
    echo "Building detection_pkg with USE_TENSORRT=ON ..."
    cd "$WS_ROOT"
    colcon build --packages-select detection_pkg --cmake-args -DUSE_TENSORRT=ON
    source "$WS_ROOT/install/setup.bash"
  fi
  "$BUILDER" --all
fi

echo "=== Done ==="
ls -lh "$SIMMODEL"/*.engine "$WS_ROOT/src/detection_pkg"/*.engine 2>/dev/null || true
