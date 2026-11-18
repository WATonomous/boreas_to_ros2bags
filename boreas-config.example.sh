# Copy to boreas-config.local.sh and edit the defaults below.
# run.sh loads the local file automatically; no exports are needed.
# Existing environment values override these defaults.
BOREAS_DATA_ROOT="${BOREAS_DATA_ROOT:-$HOME/datasets/boreas}"
BOREAS_BAG_ROOT="${BOREAS_BAG_ROOT:-$HOME/rosbags/boreas}"
BOREAS_SEQUENCE="${BOREAS_SEQUENCE:-boreas-2020-11-26-13-58}"
BOREAS_IMAGE="${BOREAS_IMAGE:-boreas_to_ros2bag:jazzy}"
