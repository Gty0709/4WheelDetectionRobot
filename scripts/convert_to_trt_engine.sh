#!/usr/bin/env bash
# Convert ONNX models to TensorRT engines using trtexec
# Usage: bash scripts/convert_to_trt_engine.sh [--fp16] [--int8]
set -euo pipefail

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIMMODEL_DIR="$WS_ROOT/src/detection_pkg/simmodel"
FP16_FLAG="--fp16"

for arg in "$@"; do
    case "$arg" in
        --int8)  FP16_FLAG="--int8" ;;
        --fp16)  FP16_FLAG="--fp16" ;;
        --fp32)  FP16_FLAG="" ;;
    esac
done

TRTEXEC=""
for candidate in /usr/local/cuda/bin/trtexec /usr/bin/trtexec; do
    if command -v "$candidate" &>/dev/null; then
        TRTEXEC="$candidate"
        break
    fi
done
if [ -z "$TRTEXEC" ]; then
    echo "trtexec not found; falling back to build_trt_engines"
    export ROBOTHOMEWORK_WS="$WS_ROOT"
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    if [ -f "$WS_ROOT/install/setup.bash" ]; then
        source "$WS_ROOT/install/setup.bash"
    fi
    BUILDER="$WS_ROOT/install/detection_pkg/lib/detection_pkg/build_trt_engines"
    if [ ! -x "$BUILDER" ]; then
        echo "ERROR: $BUILDER not found. Build with:"
        echo "  colcon build --packages-select detection_pkg --cmake-args -DUSE_TENSORRT=ON"
        exit 1
    fi
    exec "$BUILDER" --all
fi

convert_model() {
    local onnx_path="$1"
    local engine_path="${onnx_path%.onnx}.engine"
    if [ ! -f "$onnx_path" ]; then
        echo "SKIP: $onnx_path not found"
        return
    fi
    if [ -f "$engine_path" ]; then
        echo "Already exists: $engine_path"
        return
    fi
    echo "Converting $onnx_path -> $engine_path ..."
    "$TRTEXEC" --onnx="$onnx_path" --saveEngine="$engine_path" $FP16_FLAG --workspace=2048
    echo "Done: $engine_path"
}

# YOLO models
convert_model "$SIMMODEL_DIR/best.onnx"
convert_model "$WS_ROOT/src/detection_pkg/yolo26n.onnx"

# OSNet ReID model
convert_model "$SIMMODEL_DIR/osnet_ain_x0_25.onnx"

echo "All TensorRT engine conversions complete."
