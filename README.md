# Boreas to ROS 2 bag

Convert an **original [Boreas](https://www.boreas.utias.utoronto.ca/) sequence**
into a ROS 2 Jazzy bag: MCAP files, standard ROS messages, original timestamps,
static calibration TF, and `metadata.yaml`. Select LiDAR, IMU, camera, and
optional post-processed GNSS. Radar and Boreas-RT are not supported.

## 1. Set up

Requires Docker. Build the standalone image once:

```bash
git clone https://github.com/WATonomous/boreas_to_ros2bags.git
cd boreas_to_ros2bags
docker build -t boreas_to_ros2bag:jazzy .

# Change these to your data and output directories (absolute paths).
export BOREAS_DATA_ROOT="$HOME/datasets/boreas"
export BOREAS_BAG_ROOT="$HOME/rosbags/boreas"
export SEQUENCE=boreas-2020-11-26-13-58
```

Already have a sequence? Set `BOREAS_DATA_ROOT` to its **parent directory**
and `SEQUENCE` to its folder name, then skip the download.

## 2. Download a sequence

Choose a sequence from the [Boreas dataset](https://www.boreas.utias.utoronto.ca/).
Install the [AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html),
then download only the files needed for LiDAR, IMU, and optional GNSS:

```bash
mkdir -p "$BOREAS_DATA_ROOT"
aws s3 ls s3://boreas/ --no-sign-request
aws s3 sync "s3://boreas/$SEQUENCE" "$BOREAS_DATA_ROOT/$SEQUENCE" \
  --no-sign-request --exclude '*' \
  --include 'calib/*' --include 'applanix/*.csv' --include 'lidar/*'
```

No AWS account is required. For camera, append `--include 'camera/*'`.
For reported GNSS uncertainty, also append `--include 'applanix/sbet.out'`
and `--include 'applanix/smrmsg.out'`.
Re-running sync downloads missing files; selecting CSVs avoids large raw logs. See the
[official download guide](https://github.com/utiasASRL/pyboreas#download-instructions)
and [data reference](https://github.com/utiasASRL/pyboreas/blob/master/DATA_REFERENCE.md).
PyBoreas is not required. Keep `calib/` for every selection. IMU needs
`applanix/imu_raw.csv`; GNSS needs `applanix/gps_post_process.csv` (not available
in every test sequence). LiDAR and camera need their respective sensor folders.

## 3. Convert

Export a 60-second LiDAR + IMU window into a **new output directory**:

```bash
./run.sh -m boreas_to_ros2bag.convert \
  --input "$BOREAS_DATA_ROOT/$SEQUENCE" \
  --output "$BOREAS_BAG_ROOT/sample60" \
  --sensors lidar imu --start 0 --duration 60
```

Omit `--duration` to export all remaining data. For a later window, use e.g.
`--start 120 --duration 30`. To include GNSS, use `--sensors lidar imu gnss`;
to include everything supported, use `--sensors lidar imu camera gnss`.
Each sensor can also be selected alone.

**Time windows begin at the earliest timestamp among the selected sensors.**
`--start 120 --duration 30` exports `[first + 120 s, first + 150 s)`.
Sensors can start at different times; the first part of a bag may contain only
IMU. Changing `--sensors` can change the time-window origin.

### Converter arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `--input PATH` | Required | One sequence directory, not the dataset root. |
| `--output PATH` | Required | New bag directory, outside the source sequence; existing paths are refused. |
| `--sensors SENSOR ...` | `lidar imu` | Any combination of `lidar`, `imu`, `camera`, `gnss`. GNSS is post-processed ground truth. |
| `--start SECONDS` | `0` | Nonnegative offset from the earliest selected sensor timestamp. Decimal seconds accepted. |
| `--duration SECONDS` | All remaining | Positive window length in seconds. Decimal seconds accepted. |
| `--no-orientation` | Off | Write raw IMU only; skip orientation initialization and the estimated-orientation topic. |
| `--seed-seconds SECONDS` | `2` | Initial stationary IMU interval for tilt/bias estimation; greater than 0, at most 30. |
| `--stationary-gyro RAD_S` | `0.02` | Positive maximum gyro norm allowed during initialization. |
| `--raw-extrinsic PATH` | Axes-only transform | Measured 4×4 `T_applanix_raw_imu` text matrix; must be accessible inside the mounted directories. |
| `--max-bag-gib GIB` | `4` | Approximate MCAP split size in GiB; minimum `0.01`. Splits share bag metadata. |
| `--gnss-status {fix,unknown}` | `fix` | Mark validated post-processed positions valid (`0`), or leave status unknown (`-2`). This is an export convention, not recovered receiver quality. |
| `-h`, `--help` | — | Show command help. |

Orientation options apply to IMU; `--no-orientation` disables seeding.
Camera also needs `T_camera_lidar.txt` and `P_camera.txt` in `calib/`.

### Directories and runtime

`run.sh` mounts `BOREAS_DATA_ROOT` read-only and `BOREAS_BAG_ROOT` writable at
**the same absolute paths** inside the container. All input/output/extrinsic
paths must be within those mounts. Without the exports above, defaults are
this repo's `data/` and `bags/`; the output root is created automatically.
`BOREAS_IMAGE` overrides the default image `boreas_to_ros2bag:jazzy`.

For native Jazzy, install the [Dockerfile dependencies](Dockerfile), source
`/opt/ros/jazzy/setup.bash`, and replace `./run.sh` with `python3`.

## 4. Validate and play

Read every message back, check schemas/timestamps/TF, and compare LiDAR, raw
IMU, and GNSS values with the source files:

```bash
./run.sh -m boreas_to_ros2bag.validate \
  "$BOREAS_BAG_ROOT/sample60" --compare-source
```

Validator arguments: bag directory, optional `--compare-source` (requires the
original input at its recorded path), and `-h`/`--help`. Camera checks cover
layout/counts/calibration, not source pixels. Reports are saved as
`conversion.json` and `validation.json` inside the bag directory.

On a ROS 2 system with the MCAP plugin installed:

```bash
ros2 bag info "$BOREAS_BAG_ROOT/sample60"
ros2 bag play "$BOREAS_BAG_ROOT/sample60" --clock --rate 0.5 \
  --qos-profile-overrides-path config/playback_qos.yaml
```

Set consuming nodes to `use_sim_time:=true`. Playback generates `/clock`.

### Output topics

| Selection | Topic | ROS message |
| --- | --- | --- |
| `lidar` | `/boreas/lidar/points` | `sensor_msgs/msg/PointCloud2` |
| `imu` | `/boreas/imu/data_raw` | `sensor_msgs/msg/Imu` |
| `imu` without `--no-orientation` | `/boreas/imu/data` | `sensor_msgs/msg/Imu` with estimated orientation |
| `camera` | `/boreas/camera/image_rect` | `sensor_msgs/msg/Image` (`rgb8`) |
| `camera` | `/boreas/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `gnss` | `/boreas/gnss/postprocessed/fix` | `sensor_msgs/msg/NavSatFix` |
| Always | `/tf_static` | `tf2_msgs/msg/TFMessage` |

## Important gotchas

- **GNSS is fused ground truth.** Avoid using it as independent GNSS in Boreas
  benchmarks. Coordinates become degrees; ellipsoid height is reconstructed
  from the published ENU origin. Status `0` means a valid exported position;
  satellite service remains unknown. Covariance uses aligned SMRMSG errors
  when both auxiliary files exist; otherwise it stays unknown. Clock/position
  mismatches fail conversion. [Source details](docs/BOREAS.md#ellipsoid-height-and-uncertainty).
- **IMU orientation needs a stationary start.** The initial two seconds seed
  tilt/bias; yaw is arbitrary and drifts. Later windows integrate preceding
  samples. Use `--no-orientation` if seeding fails. Processed `imu.csv` has
  gravity removed and is not used as raw IMU.
- **Frames and timing matter.** Epoch timestamps stay unchanged. LiDAR `time`
  is signed seconds from scan center; `ring` is UINT16. Raw IMU axes are
  backward/left/down; its default origin is assumed co-located with Applanix.
  Supply a measured `--raw-extrinsic` when needed. [Frame details](docs/DATA.md).
- **Keep the whole output directory.** MCAP chunks use Zstd, but decoded camera
  data can be large. Keep all splits and `metadata.yaml`. Interrupted runs may
  leave partial bags; inspect them and use a new output path to retry.

[Verified dataset notes and sources](docs/BOREAS.md) ·
[Detailed data conventions](docs/DATA.md)
