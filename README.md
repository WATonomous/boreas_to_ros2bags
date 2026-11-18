# boreas_to_ros2bag

Convert an original Boreas sequence into a **ROS 2 Jazzy MCAP/CDR bag** with
`metadata.yaml`, standard ROS messages, preserved acquisition timestamps and
static calibration TF. Uses the native writer approach from WATonomous's
nuScenes converter; [the source comparison and reuse decision](docs/RESEARCH.md)
explain why its nuScenes reader and other Boreas converters were not forked.

The repository is `/home/watonomous/jsteph/boreas_to_ros2bag`, separate from
`wato_monorepo`. Raw data is read only; generated bags live on `/mnt/data`.

## Convert and inspect

The existing world-modeling image already has Jazzy, NumPy and the MCAP plugin.
`run.sh` uses a disposable, network-disabled container; it does not start, stop
or rebuild any watod service. No pip installation is needed. On the host:

```bash
cd /home/watonomous/jsteph/boreas_to_ros2bag
./run.sh -m unittest discover -s tests -v
./run.sh -m boreas_to_ros2bag.convert \
  --input /mnt/data/boreas/boreas-2020-11-26-13-58 \
  --output /mnt/data/boreas/bags/boreas_sample45 \
  --sensors lidar imu --duration 45
./run.sh -m boreas_to_ros2bag.validate \
  /mnt/data/boreas/bags/boreas_sample45 --compare-source
```

**The sample above already exists.** The converter refuses overwrites; choose a
new output directory to repeat conversion. `--start` and `--duration` are seconds
from the earliest selected sensor timestamp, with a half-open interval. The raw
IMU begins about 20 seconds before the first LiDAR scan, so this sample has
20 seconds of IMU initialization followed by about 25 seconds of LiDAR, including
vehicle motion. Later windows still integrate the preceding raw IMU samples,
but Eidos itself should start on a stationary window for LISO warmup.

Native `ros2 bag info` without starting the stopped watod container:

```bash
docker run --rm --network none --entrypoint bash \
  -v /mnt/data/boreas/bags:/bags:ro \
  ghcr.io/watonomous/wato_monorepo/world_modeling/world_modeling:dep_eidos-tro \
  -c 'source /opt/ros/jazzy/setup.bash; ros2 bag info /bags/boreas_sample45'
```

For a different data root, set `BOREAS_DATA_ROOT` and `BOREAS_BAG_ROOT` before
`run.sh`; paths in `--input`/`--output` must be within those mounted directories.
The converter keeps one cloud/image per active stream plus filename indexes,
a two-second IMU seed, and a 4 MiB MCAP chunk in memory. It never loads the
sequence's clouds or CSV in full. Files split at approximately 4 GiB; all splits
share bag metadata. MCAP uses Zstd chunk compression, not whole-file compression.
`conversion.json` records selection, counts, bias estimate and frame assumptions.
`validation.json` records readback checks. A failed/interrupted run can leave a
partial directory; use a new output path and inspect it before retaining it.

## Eidos playback

At inspection, `watod_watonomous-world_modeling_bringup_dev-1` was **exited**.
It was left untouched. Its current `/ws/bags` bind points at Lucas's bag path,
not `/mnt/data`. At your next planned container start, set this line in
`/home/watonomous/jsteph/wato_monorepo/watod-config.local.sh`:

```bash
export BAG_DIRECTORY="/mnt/data/boreas/bags"
```

Then run the following on the host **when ready to recreate/start that service**;
changing a bind mount requires recreation. Rebuild Eidos yourself if needed.
The new configuration is copied as a small file, without changing existing
Ring Road configs or requiring a config-only rebuild:

```bash
cd /home/watonomous/jsteph/wato_monorepo
./watod up -d
docker cp /home/watonomous/jsteph/boreas_to_ros2bag/config/eidos_boreas_slam.yaml \
  watod_watonomous-world_modeling_bringup_dev-1:/tmp/eidos_boreas_slam.yaml
docker cp /home/watonomous/jsteph/boreas_to_ros2bag/config/playback_qos.yaml \
  watod_watonomous-world_modeling_bringup_dev-1:/tmp/boreas_playback_qos.yaml
./watod exec world_modeling_bringup_dev bash
```

In container terminal 1:

```bash
source /ws/install/setup.bash
ros2 launch eidos_transform eidos_transform.launch.yaml \
  config_file:=/tmp/eidos_boreas_slam.yaml use_sim_time:=true
```

In container terminal 2 (enter with `./watod exec world_modeling_bringup_dev bash`):

```bash
source /ws/install/setup.bash
ros2 launch eidos eidos.launch.yaml \
  eidos_config_file:=/tmp/eidos_boreas_slam.yaml use_sim_time:=true
```

In container terminal 3, start from the beginning after both nodes are active:

```bash
source /ws/install/setup.bash
ros2 bag info /ws/bags/boreas_sample45
ros2 bag play /ws/bags/boreas_sample45 --clock --rate 0.5 \
  --qos-profile-overrides-path /tmp/boreas_playback_qos.yaml \
  --topics /boreas/lidar/points /boreas/imu/data /tf_static
```

The bag already records the correct static QoS; this override makes the playback
choice explicit. Let the player generate `/clock`. Topics and frames are Boreas
specific; the supplied Eidos config refers to them directly without pretending
to be Novatel or Ring Road. It enables LISO only, no GPS factor or GPS
relocalization plugins, and supplies the orientation LISO needs for initialization.
It fuses LiDAR odometry and IMU gyro in the transform EKF; acceleration and EKF
orientation fusion are disabled. See [IMU signs and calibration assumptions](docs/DATA.md).

Check `IMU warmup complete`, `TRACKING` and increasing keyframes:

```bash
ros2 topic echo /world_modeling/slam/status
ros2 run tf2_ros tf2_echo boreas_base boreas_lidar
```

In Foxglove use fixed frame `map`, cloud `/boreas/lidar/points`, map
`/world_modeling/slam/visualization/map`, odometry
`/world_modeling/transform/odometry` and `/tf`. A map can be saved explicitly:

```bash
ros2 service call /world_modeling/slam/save_map eidos_msgs/srv/SaveMap \
  "{filepath: '/opt/watonomous/maps/boreas.map'}"
```

This is a SLAM baseline. GPS-assisted localization is deliberately absent;
localizing a saved map requires an appropriate non-GPS initial-pose/relocalization
strategy, not a fabricated GPS fix or a ground-truth seed.

## Full sequence and optional camera

After the validated sample, convert all downloaded LiDAR and IMU measurements:

```bash
cd /home/watonomous/jsteph/boreas_to_ros2bag
./run.sh -m boreas_to_ros2bag.convert \
  --input /mnt/data/boreas/boreas-2020-11-26-13-58 \
  --output /mnt/data/boreas/bags/boreas-2020-11-26-13-58-lidar-imu \
  --sensors lidar imu
```

The full LiDAR/IMU bag above **already exists and has passed source comparison**.
It contains 9,984 scans, 215,097 raw IMU messages, 215,097 IMU messages with
estimated orientation and one static TF message: 440,179 records over
1,075.4793698 seconds. Native Jazzy `ros2 bag info` reports 32.4 GiB across nine
MCAP files. `validation.json` in the bag directory records successful CDR/schema,
timestamp, TF/QoS and complete source-data checks. Camera is not included.
Choose a new output path to repeat conversion; the existing bag is never overwritten.
Eidos playback testing is still pending.

For a portable Jazzy environment or camera decoding, build the converter image
(separate from Eidos). This installs ROS bag packages, NumPy and Pillow:

```bash
cd /home/watonomous/jsteph/boreas_to_ros2bag
docker build -t boreas_to_ros2bag:jazzy .
BOREAS_IMAGE=boreas_to_ros2bag:jazzy ./run.sh -m boreas_to_ros2bag.convert \
  --input /mnt/data/boreas/boreas-2020-11-26-13-58 \
  --output /mnt/data/boreas/bags/boreas_camera_sample \
  --sensors lidar imu camera --duration 22
```

Camera output is decoded `sensor_msgs/msg/Image` (`rgb8`) plus rectified
`CameraInfo` and optical-frame TF. It is optional and increases bag size.
For native Jazzy installations, source `/opt/ros/jazzy/setup.bash`, install the
packages listed in `Dockerfile`, and run `python3 -m boreas_to_ros2bag.convert`
from the repo instead of `run.sh`.

## Verified boundary

The initial 45-second bag is 759.3 MiB with 18,245 messages: 242 PointCloud2,
9,001 raw Imu, 9,001 Imu with estimated orientation, one TFMessage. Six unit
tests pass on Jazzy. Native readback checks every CDR message, counts, timestamps,
embedded ROS schemas, TF tree/QoS, and compares every cloud field and raw IMU
value against source. An independent `rosbags` reader agrees on counts and order.
See [sample verification](docs/SAMPLE.md).

Optional camera decoding, dimensions, calibration and standard-message CDR round
trips were checked on the host's Humble/Pillow environment. A Jazzy camera bag
and the portable Dockerfile build have not been tested. **Eidos rebuild, playback,
tracking quality and Foxglove display remain for your runtime testing.**
