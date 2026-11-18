#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$repo_dir/boreas-config.local.sh" ]]; then
  source "$repo_dir/boreas-config.local.sh"
fi
image="${BOREAS_IMAGE:-boreas_to_ros2bag:jazzy}"
data_root="${BOREAS_DATA_ROOT:-$repo_dir/data}"
bag_root="${BOREAS_BAG_ROOT:-$repo_dir/bags}"
data_root="$(cd -- "$data_root" && pwd)"
mkdir -p "$bag_root"
bag_root="$(cd -- "$bag_root" && pwd)"
exec docker run --rm --network none --user "$(id -u):$(id -g)" \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e "BOREAS_DATA_ROOT=$data_root" \
  -e "BOREAS_BAG_ROOT=$bag_root" \
  -e "BOREAS_SEQUENCE=${BOREAS_SEQUENCE:-}" \
  -v "$repo_dir:/converter:ro" \
  -v "$data_root:$data_root:ro" \
  -v "$bag_root:$bag_root:rw" \
  -w /converter --entrypoint bash "$image" \
  -c 'source /opt/ros/jazzy/setup.bash; exec python3 "$@"' bash "$@"
