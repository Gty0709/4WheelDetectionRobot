#!/usr/bin/env bash
# 回放 maps/bags/slam_20260613_151154 的 cmd_vel，双目录包 → maps/new_with_camera/
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAPS_DIR="$ROOT/src/perception_pkg/maps/new_with_camera"
REF_BAG="$ROOT/src/perception_pkg/maps/bags/slam_20260613_151154"

_cleanup() {
  if [[ -f "$MAPS_DIR/_cache/latest.pgm" ]]; then
    echo "[auto_build_map_camera] 退出兜底：从缓存保存地图..."
    # shellcheck source=/dev/null
    source "$ROOT/env_humble.bash" 2>/dev/null || true
    ros2 run perception_pkg save_map_snapshot.py --from-cache --maps-dir "$MAPS_DIR" 2>/dev/null || \
      python3 "$ROOT/src/perception_pkg/scripts/save_map_snapshot.py" --from-cache --maps-dir "$MAPS_DIR" || true
  fi
}
trap _cleanup INT TERM EXIT

mkdir -p "$MAPS_DIR/bags"
# shellcheck source=/dev/null
source "$ROOT/env_humble.bash"
exec ros2 launch perception_pkg auto_slam_camera.launch.py \
  replay_cmd_vel_bag:="$REF_BAG" "$@"
