#!/usr/bin/env bash
# 一键全自动建图；Ctrl-C 时从缓存兜底保存地图
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAPS_DIR="$ROOT/src/perception_pkg/maps"

_cleanup() {
  if [[ -f "$MAPS_DIR/_cache/latest.pgm" ]]; then
    echo "[auto_build_map] 退出兜底：从缓存保存地图..."
    # shellcheck source=/dev/null
    source "$ROOT/env_humble.bash" 2>/dev/null || true
    ros2 run perception_pkg save_map_snapshot.py --from-cache --maps-dir "$MAPS_DIR" 2>/dev/null || \
      python3 "$ROOT/src/perception_pkg/scripts/save_map_snapshot.py" --from-cache --maps-dir "$MAPS_DIR" || true
  fi
}
trap _cleanup INT TERM EXIT

# shellcheck source=/dev/null
source "$ROOT/env_humble.bash"
exec ros2 launch perception_pkg auto_slam.launch.py "$@"
