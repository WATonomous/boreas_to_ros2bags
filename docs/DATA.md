# Boreas conventions and estimator boundary

Sources and the converter comparison are in [RESEARCH.md](RESEARCH.md).

## Files and timestamps

This is the original Boreas sequence, not Boreas-RT. Locally inspected:
9,984 LiDAR binaries, 20,624 rectified camera PNGs, 215,097 raw IMU rows.
The raw IMU maximum time step across the complete CSV is 0.0050008 s.

LiDAR is little-endian float32 `(N,6)`: x, y, z in metres, intensity, laser ID,
and time in **seconds relative to the filename timestamp**. The first scan has
213,346 points, rings 0–127, intensity 0–203, and relative times about
[-0.051758, +0.051758] s. Intensity remains unnormalized float32. Ring becomes
UINT16 without changing its value. `time` remains signed FLOAT32 seconds relative
to the middle of the scan. It is not nanoseconds, not a start-of-scan offset,
and not an absolute epoch. No deskewing is performed; LISO currently ignores ring/time.

Sensor filenames and sensor pose CSV epochs are microseconds UTC since Unix epoch.
`imu_raw.csv` uses decimal **seconds** despite the `GPSTime` header. The first
`ros_and_gps_time.csv` pair differs by about 21 microseconds, not 18 seconds;
the first 1,000 pairs range from -0.349 ms to +7.007 ms. The recording computer
and LiDAR were UTC synchronized through Applanix. We preserve the dataset's
epochs and do not apply an additional GPS leap-second correction or replace
them with computer receipt times. Decimal parsing retains the raw CSV's 100 ns
precision. Camera filenames already identify the middle of exposure; no second
event-time correction is applied. The CSV tables remain untouched.

Every record time equals its sensor header time. Independent sensor generators
are merged by timestamp. A scan is stamped at its temporal middle, so some of
its points precede the bag record time. `/tf_static` is one aggregate message at
the first emitted sensor time; it includes every sensor edge and has reliable,
transient-local durability. No `/clock`, moving vehicle TF, pose, GPS fix, or
ground-truth message is written. Playback supplies `/clock`; Eidos owns map/odom TF.

## IMU and orientation

- `imu_raw.csv`: measured angular velocity in rad/s and acceleration in m/s²,
  body axes **x backward, y left, z down**. Its stationary z is approximately
  -9.8 m/s²: specific force points up, as required by ROS IMU semantics.
- `imu.csv`: **post-processed** Applanix solution, gravity removed. It is not raw
  IMU. It is not used here.
- `gps_post_process.csv`, `lidar_poses.csv`, `camera_poses.csv`: post-processed
  ground truth, excluded from all converter estimator paths. Exporting them as
  GPS or seeding initialization from their poses would leak the reference solution.

`/boreas/imu/data_raw` preserves the raw CSV values and has
`orientation_covariance[0] = -1`. `/boreas/imu/data` preserves the same gyro and
acceleration and adds an **estimate**, not an Applanix orientation:

1. Validate the first two seconds are stationary: all gyro norms <0.02 rad/s,
   mean gravity magnitude 9.3–10.3 m/s², per-axis acceleration std <0.35 m/s².
2. Align mean raw specific force to world +Z; choose initial vehicle yaw zero.
3. Estimate constant gyro bias from that interval; integrate body gyro using
   quaternion increments and trapezoidal rates. Normalize every step; fail on
   IMU gaps >0.1 s. Propagation reads earlier IMU rows for later time windows.

Orientation is marked unavailable until the seed interval ends. The 0.0025 rad²
tilt variances are approximate; yaw variance is 1e6 rad² because heading is
unobservable. Orientation drifts with gyro error during motion; it is not a
general calibrated attitude solution. Eidos uses it for initial tilt only and
uses raw gyro thereafter. Its EKF has `use_orientation: false`.

LISO requires a valid quaternion **and** consecutive low-gyro samples. Disabling
EKF orientation alone cannot initialize it. The Boreas config uses 200 samples
and a 0.02 rad/s threshold to accommodate observed stationary gyro noise.
The sample has 4,881 consecutive valid stationary orientation samples. If a new
sequence does not start stationary, conversion fails; `--no-orientation` exports
raw IMU only and is explicitly unsuitable for the supplied LISO configuration.
The measured initial gravity is 9.80285 m/s²; the config rounds this to 9.803.

Eidos currently calculates acceleration compensation as `acc_base - g_body`
with downward `g_world`. ROS raw specific force is upward when stationary.
Enabling its current compensation on these messages would double gravity.
The supplied config disables EKF **linear acceleration fusion**, retains
gyro/LiDAR, and sets `gravity_compensated: false`. This does not alter the raw
bag values. Enabling acceleration needs an estimator-side sign review and
raw-IMU lever-arm confirmation; copying Ring Road's already-compensated setting
would be incorrect.

## Calibration and frame tree

`T_A_B` maps coordinates **from B into A**, also the pose of child B in parent A.
`T_applanix_lidar.txt` is used directly. The camera edge is
`inverse(T_camera_lidar.txt)`, with the camera optical axes right/down/forward.
The dataset PNGs are already rectified: CameraInfo uses the first 3x4 block of
`P_camera.txt`, K = its left 3x3, R = identity, D = zero. Raw calibration
distortion from `camera0_intrinsics.yaml` must not be applied a second time.

```text
boreas_base                    (chosen vehicle FLU frame, Applanix origin)
  └── boreas_applanix           (x right, y forward, z up)
        ├── boreas_lidar        (dataset coordinates, calibrated +0.13 m z)
        │     └── boreas_camera_optical  (only with camera selected)
        └── boreas_imu_raw      (x backward, y left, z down)
```

The two axis changes are proper rotations:

```text
R_base_applanix = [[ 0, 1, 0], [-1, 0, 0], [0, 0, 1]]
R_applanix_raw  = [[ 0,-1, 0], [-1, 0, 0], [0, 0,-1]]
R_base_raw     = diag(-1, 1, -1)
```

The default raw-IMU edge assumes a shared origin with the Applanix reference:
the original dataset does not provide a separate raw-IMU lever arm. Its
**rotation is established from documented axes; its translation is an explicit
unverified assumption**, not a measured calibration. Supply
`--raw-extrinsic /path/T_applanix_raw_imu.txt` when that measurement is available.
The baseline uses only IMU orientation and angular velocity, both independent
of that translation. Do not import the newer Boreas-RT DMU extrinsics.

The validator checks a unique-parent acyclic tree rooted at `boreas_base`, unit
quaternions, stored transform values and original LiDAR/camera calibration files.
Physical calibration accuracy and Eidos tracking still require runtime testing.
