#!/bin/bash
# 仅安装 deps-apt.list 中缺失的 deb 包
set -e
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DEPS_FILE="$WS_DIR/config/humble/deps-apt.list"
TO_INSTALL=()

while IFS= read -r pkg || [ -n "$pkg" ]; do
  pkg="${pkg%%#*}"
  pkg="$(echo "$pkg" | tr -d '\r' | xargs)"
  [ -z "$pkg" ] && continue
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then
    TO_INSTALL+=("$pkg")
  fi
done < "$DEPS_FILE"

if [ ${#TO_INSTALL[@]} -eq 0 ]; then
  echo "[install_deps] 无缺失包，跳过 apt"
  exit 0
fi

echo "[install_deps] 将安装: ${TO_INSTALL[*]}"
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${TO_INSTALL[@]}"
echo "[install_deps] 完成"
