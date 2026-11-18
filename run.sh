#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image="${BOREAS_IMAGE:-ghcr.io/watonomous/wato_monorepo/world_modeling/world_modeling:dep_eidos-tro}"
data_root="${BOREAS_DATA_ROOT:-/mnt/data/boreas}"
bag_root="${BOREAS_BAG_ROOT:-/mnt/data/boreas/bags}"
mkdir -p "$bag_root"
exec docker run --rm --network none --user "$(id -u):$(id -g)" \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$repo_dir:/converter:ro" \
  -v "$data_root:$data_root:ro" \
  -v "$bag_root:$bag_root:rw" \
  -w /converter --entrypoint bash "$image" \
  -c 'source /opt/ros/jazzy/setup.bash; exec python3 "$@"' bash "$@"
