#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_find_ws_root() {
  local dir="$1"
  while [ -n "$dir" ] && [ "$dir" != "/" ]; do
    if [ -f "$dir/scripts/setup_detection_venv.sh" ] || [ -f "$dir/env_humble.bash" ]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

WS_ROOT="$(_find_ws_root "$SCRIPT_DIR" || true)"
if [ -z "$WS_ROOT" ]; then
  echo "[clip_detector] 无法定位工作区根目录（从 $SCRIPT_DIR 向上查找）" >&2
  exit 1
fi

TARGET="$WS_ROOT/.detection-python"
NODE_PY="$SCRIPT_DIR/clip_detector_node.py"

if [ ! -f "$TARGET/.installed" ]; then
  echo "[clip_detector] 缺少隔离层: $TARGET" >&2
  echo "[clip_detector] 请执行: bash $WS_ROOT/scripts/setup_detection_venv.sh" >&2
  exit 1
fi

export DETECTION_PYTHON_TARGET="$TARGET"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$WS_ROOT/install/setup.bash"
fi

exec python3 "$NODE_PY" "$@"
