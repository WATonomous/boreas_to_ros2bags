# Sample validation

Bag: `/mnt/data/boreas/bags/boreas_sample45` (759.3 MiB).

| Topic | ROS 2 type | Count |
| --- | --- | ---: |
| `/boreas/lidar/points` | `sensor_msgs/msg/PointCloud2` | 242 |
| `/boreas/imu/data_raw` | `sensor_msgs/msg/Imu` | 9,001 |
| `/boreas/imu/data` | `sensor_msgs/msg/Imu` | 9,001 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 |

18,245 records over 44.999986 seconds. First timestamp:
`1606417077521302200` ns; last: `1606417122521288200` ns. All sensor header times
equal record times. The IMU is about 200 Hz; the LiDAR about 9.6 Hz. The first
20 seconds contain IMU only, before LiDAR recording begins.

Verified on Jazzy with native `ros2 bag info` and `SequentialReader`:

- MCAP storage, metadata version 9, native CDR serialization and embedded
  `ros2msg` definitions for every message type.
- All 18,245 records deserialize; timestamps are nondecreasing.
- Every source point's x/y/z, intensity, ring and relative time matches the bag.
  No filtering, normalization or artificial point times were introduced.
- Every raw IMU timestamp, angular velocity and acceleration matches the CSV.
- Metadata counts match decoded records and source files/rows in the interval.
- Three static edges form one connected acyclic tree; unit quaternions match
  transform matrices and the LiDAR calibration file. Static QoS is reliable and
  transient local. The physical raw-IMU origin remains an explicit assumption.
- 400 initial derived IMU samples mark orientation unavailable while collecting
  the two-second seed. There are 4,881 consecutive stationary, valid-orientation
  samples afterward, exceeding LISO's configured 200-sample threshold.
- An independent host `rosbags.rosbag2.Reader` agrees on CDR topic types, counts
  and chronology.

The last scan has 217,561 points, ring range 0–127 and point-time range
[-0.05188036, +0.05187941] seconds relative to its header.

Six focused tests pass on both the host and Jazzy: exact timestamp parsing,
nonzero tilt/gravity alignment, gyro propagation, half-turn frame rotation,
point field preservation and malformed cloud rejection.

The optional camera path was checked separately against an actual PNG:
2448x2048 RGB, 15,040,512 decoded bytes, rectified CameraInfo, inverse camera
extrinsic, Image/CameraInfo CDR round trips under the host's Humble/Pillow.
No Jazzy camera bag was generated and the portable Dockerfile was not built.

No Eidos nodes or playback were started, and no map was built. Successful LISO
warmup, tracking performance and Foxglove visualization await the user's test.
