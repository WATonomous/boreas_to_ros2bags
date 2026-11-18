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
and not an absolute epoch. No deskewing is performed.

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
transient-local durability. No `/clock`, moving vehicle TF, or pose is written.
Playback supplies `/clock`; consuming applications own map/odom TF.
Post-processed GNSS is exported only when explicitly selected with `--sensors gnss`.

## Optional GNSS

`--sensors gnss` reads `applanix/gps_post_process.csv` and writes
`sensor_msgs/msg/NavSatFix` on `/boreas/gnss/postprocessed/fix`. The navigation
solution is post-processed GNSS/INS ground truth, not independent raw GNSS.
The converter records `ground_truth_used: true` and the source and limitations
in `conversion.json`. Other sensor selections do not read this file.

Latitude/longitude are converted from radians to degrees. The source coordinates
use ITRF2000 at the observation epoch on the WGS84 ellipsoid; no realization
or epoch transformation is performed. Ellipsoid height is reconstructed by
mapping the CSV's east/north/up position into ECEF using the published shared
Boreas origin, then projecting onto the geodetic normal at the reported
latitude/longitude. A residual above 1 cm rejects incompatible coordinates.
This accounts for Earth curvature; local up is never used directly as altitude.

By default `status=0` (`STATUS_FIX`) is a validity convention for the validated
exported position, not an observation of the receiver's correction mode.
`--gnss-status unknown` retains `status=-2`. Neither setting claims RTK, SBAS,
GBAS, or independent raw GNSS. The service mask remains zero (unknown).

When `applanix/sbet.out` and `applanix/smrmsg.out` are both available, the
converter verifies every CSV timestamp and latitude/longitude against SBET
using UTC seconds-of-week. It refuses an inferred GPS/UTC leap-second offset,
position mismatches, or missing uncertainty coverage. Interpolated SMRMSG
north/east/down position RMS values are squared and reordered into the ROS
east/north/up covariance diagonal. Cross terms are unavailable; zeros occupy
those entries and the covariance type is `APPROXIMATED` (1). Reported solution
uncertainty does not independently establish centimetre-accurate ground truth.
If either auxiliary file is missing, covariance stays zero with type `UNKNOWN`
(0); the absence is recorded in `conversion.json`. Malformed or inconsistent
auxiliary files fail conversion rather than silently supplying unreliable data.

The header frame is `boreas_applanix`, the navigation reference point, not an
assumed antenna location. Header and bag timestamps match the exact decimal CSV
epoch. Missing CSVs or malformed columns, coordinates, and non-increasing
timestamps fail conversion. See [official sources and format checks](BOREAS.md).

## IMU and orientation

- `imu_raw.csv`: measured angular velocity in rad/s and acceleration in m/s²,
  body axes **x backward, y left, z down**. Its stationary z is approximately
  -9.8 m/s²: specific force points up, as required by ROS IMU semantics.
- `imu.csv`: **post-processed** Applanix solution, gravity removed. It is not raw
  IMU. It is not used here.
- `gps_post_process.csv`: optional post-processed GNSS export described above.
  It is never used to initialize IMU orientation. `lidar_poses.csv` and
  `camera_poses.csv` are not used. Fusing the GNSS export when benchmarking
  against the same reference solution would leak ground truth.

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
general calibrated attitude solution. If a sequence does not start stationary,
conversion fails; `--no-orientation` exports raw IMU only. Check whether the
consuming application requires an orientation estimate. Raw acceleration is
specific force, not gravity-compensated acceleration; preserve that distinction
when configuring sensor fusion.

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
Physical calibration accuracy and downstream tracking still require runtime testing.
