# Original Boreas data reference

Verified 2026-11-17 against the original Boreas release and official
`utiasASRL/pyboreas` revision `8e100ff82ecd4992511da61b016398dbd230f205`.
These instructions concern original Boreas; Boreas-RT adds different sensors.
The [official devkit README](https://github.com/utiasASRL/pyboreas/blob/8e100ff82ecd4992511da61b016398dbd230f205/README.md)
describes both datasets.

## Download

Install the [AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html).
An AWS account is optional: use `--no-sign-request`. The
[official download instructions](https://github.com/utiasASRL/pyboreas/blob/8e100ff82ecd4992511da61b016398dbd230f205/README.md#download-instructions)
support downloading individual sequences and sensors:

```bash
root=/path/to/data/boreas
sequence=boreas-2020-11-26-13-58
mkdir -p "$root"
aws s3 ls s3://boreas/ --no-sign-request
aws s3 sync "s3://boreas/$sequence" "$root/$sequence" --no-sign-request \
  --exclude '*' --include 'calib/*' --include 'applanix/*.csv' \
  --include 'lidar/*' --include 'camera/*'
```

This is an adaptation of the official selection example. Keep only the sensor
directories needed: `lidar/` for LiDAR, `camera/` for camera. CSVs under
`applanix/` supply IMU and available processed GNSS; `calib/` supplies sensor
calibration. Selecting CSVs avoids downloading large `applanix/raw_logs/`
binaries and POSPac outputs. `radar/` is a separate sensor directory; its PNGs
are radar measurements. To download the entire release, the official command is
`aws s3 sync s3://boreas "$root" --no-sign-request`. The optional official reader
installs with `pip install asrl-pyboreas`; this converter does not require it.

## GNSS availability and actual CSV

The official reference identifies `applanix/gps_post_process.csv` as the 200 Hz
Applanix ground truth obtained by batch processing GNSS, IMU, wheel encoder and
RTX corrections. It is a fused reference solution, unsuitable as an independent
raw GNSS input when evaluating against Boreas ground truth.
[Official data reference](https://github.com/utiasASRL/pyboreas/blob/8e100ff82ecd4992511da61b016398dbd230f205/DATA_REFERENCE.md#pose-files).

The actual [baseline CSV](https://boreas.s3.amazonaws.com/boreas-2020-11-26-13-58/applanix/gps_post_process.csv)
and [2021-08-05 CSV](https://boreas.s3.amazonaws.com/boreas-2021-08-05-13-34/applanix/gps_post_process.csv)
have this header:

```text
GPSTime,easting,northing,altitude,vel_east,vel_north,vel_up,roll,pitch,heading,angvel_z,angvel_y,angvel_x,accelz,accely,accelx,latitude,longitude
```

- `GPSTime` is decimal epoch **seconds** in these CSVs, e.g.
  `1606417077.5213022`; it is not the microsecond filename format. Preserve its
  decimal precision and do not apply an inferred GPS/UTC leap-second shift.
- Latitude/longitude are **radians**, confirmed by the
  [dataset author](https://github.com/utiasASRL/pyboreas/issues/56#issuecomment-2474949701).
  Convert them to degrees for ROS.
- `easting,northing,altitude` are local ENU coordinates in metres. The baseline
  first `altitude` is approximately `0.000030`, demonstrating that this column
  is local up, not absolute ellipsoid height. The published origin has
  ellipsoid height `153.67129211034626` m; adding that constant to local up is
  not an exact geodetic conversion. The author separately confirms
  [ITRF2000, WGS84 ellipsoid, observation epoch](https://github.com/utiasASRL/pyboreas/issues/91#issuecomment-5483218423),
  correcting an earlier ITRF2014 answer in the same issue.
- There are no receiver status, constellation mask, satellite counts, or
  covariance fields in this CSV. Do not turn the dataset's typical accuracy
  into per-sample covariance or claim RTK/SBAS/GBAS status.

Absolute ellipsoid height is available separately in `applanix/sbet.out`; the
dataset author's [origin reconstruction](https://github.com/utiasASRL/pyboreas/issues/91)
identifies a height near 153.671 m. Local inspection confirms a 17-double SBET
record with height 153.67132212594151 m at a position matching the baseline
CSV's first latitude/longitude. Height is also recoverable from the CSV's full
ENU position and published origin, as described below.

No `gps_raw.csv` occurs in either inspected official
[baseline listing](https://boreas.s3.amazonaws.com/?list-type=2&prefix=boreas-2020-11-26-13-58%2Fapplanix%2F)
or [2021-08-05 listing](https://boreas.s3.amazonaws.com/?list-type=2&prefix=boreas-2021-08-05-13-34%2Fapplanix%2F).
They contain `raw_logs/`, `sbet.out`, and `smrmsg.out`. This inspection does not
establish a public raw GNSS CSV schema. Test sequences may omit processed
ground truth, as the [official README](https://github.com/utiasASRL/pyboreas/blob/8e100ff82ecd4992511da61b016398dbd230f205/README.md#example-usage)
explains; require the selected file explicitly.

For a `NavSatFix` export, identify the processed source and convert
latitude/longitude to degrees. ROS requires altitude above the WGS84 ellipsoid
and explicitly permits NaN when unavailable; it requires covariance in local
ENU order, in square metres.
[NavSatFix definition](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatFix.msg).
The original receiver status and constellation mask are unavailable in these
processed CSVs. ROS interprets status >= 0 as valid; its `STATUS_FIX=0` comment
describes an unaugmented fix. No enum specifically describes a smoothed POSPac
solution. Assigning 0 as validity of an exported processed position is therefore
a converter convention, not recovered receiver quality. Do not infer SBAS,
GBAS or RTK status. A service mask of zero preserves unknown constellations.
Jazzy defines unknown status as `-2`; Humble has no named unknown constant.
[Humble NavSatStatus](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatStatus.msg),
[Jazzy NavSatStatus](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/NavSatStatus.msg).

## Ellipsoid height and uncertainty

Original Boreas uses one fixed ENU reference aligned with the first pose of
`boreas-2020-11-26-13-58`, rather than a new origin for every sequence.
The [official reference](https://github.com/utiasASRL/pyboreas/blob/8e100ff82ecd4992511da61b016398dbd230f205/DATA_REFERENCE.md#pose-files)
defines that frame; the author publishes its exact extraction constant:

```text
latitude = 0.7641426285531766 rad
longitude = -1.3869452891985545 rad
ellipsoid_height = 153.67129211034626 m
```

[Author's extraction origin](https://github.com/utiasASRL/pyboreas/issues/91#issuecomment-5464639845).
Consequently, inverse ENU conversion via ECEF and the WGS84 ellipsoid can
recover geodetic height from all three CSV position columns. This is a
geometric deduction from the published frame, not a claim that local up is
ellipsoid height. Check the reconstructed latitude/longitude against the CSV
to detect an incompatible origin or coordinate convention.

The [NOAA-maintained POSPac reader](https://github.com/noaa-ocs-hydrography/drivers/blob/master/HSTB/drivers/sbet.py)
reads SBET as 17 float64 values per record, beginning with seconds-of-week,
latitude/longitude in radians, and height in metres. It reads SMRMSG as 10
float64 values: time, north/east/down position RMS errors in metres,
north/east/down velocity RMS errors in m/s, and roll/pitch/heading RMS errors in
arcminutes. NOAA labels the position errors as 1-sigma and interpolates SMRMSG
onto SBET. The
[official Boreas error plotter](https://github.com/utiasASRL/pyboreas/blob/8e100ff82ecd4992511da61b016398dbd230f205/pyboreas/vis/plot_processed_error.py)
independently reads this ten-double order and plots position errors in metres.

After time alignment, the reported position RMS values support a diagonal ROS
covariance `[east_rms², north_rms², down_rms²]`. The converter linearly
interpolates RMS values before squaring and labels the resulting covariance
`COVARIANCE_TYPE_APPROXIMATED` (1). Down-to-up negation does not change the
variance. These are estimated solution uncertainties; SMRMSG provides no
position cross covariances. If an auxiliary file is absent, the converter
retains unknown covariance; present but malformed/unaligned files fail
conversion. A dataset-wide accuracy figure is not a per-sample covariance.

Time reference must be verified: SBET exporters can produce GPS or UTC time,
as documented by [NovAtel's exporter](https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/Export_to_SBET.htm)
and [SBG's exporter](https://support.sbg-systems.com/sc/qd/3.2/reference-manual/export-module).
For the locally inspected baseline sequence, every one of the 215,097 CSV
latitude/longitude pairs matches SBET rows 27,293 through 242,389 exactly when
`sbet_time = csv_epoch_seconds - 1606003200` (UTC Sunday 2020-11-22 00:00).
The exploratory float64 comparison has a maximum residual of 0.1193
microseconds; the converter's exact-decimal epoch parsing gives a maximum
residual of 0.2378 microseconds. Both are below its 1-microsecond alignment
tolerance. There is no additional 18-second shift in this correspondence.
Its 1,280 SMRMSG records run from 413741 to 415020 in one-second steps and
cover the entire CSV. This verifies the baseline correspondence empirically;
other sequences should be checked before applying that time convention.
To download these auxiliary files, add `--include 'applanix/sbet.out'` and
`--include 'applanix/smrmsg.out'` to the selective sync command above.

## Timing and sensor cautions

- Sensor files use UTC microseconds since Unix epoch. Camera timestamps are
  exposure midpoints; LiDAR and radar timestamps are scan midpoints. Sensors
  are asynchronous: do not equate frame indices or assume matching start/end
  times. Use actual timestamps for time windows.
- LiDAR stores float32 `[x,y,z,intensity,laser_id,time]`; the last value is a
  seconds offset added to the filename time by the official loader.
- Camera PNGs are rectified; use `P_camera.txt`. Polar radar PNGs include
  metadata in their first 11 columns, so ordinary image decoding alone does not
  produce a camera image or Cartesian radar scan.
- `imu_raw.csv` uses body axes x backward, y left, z down. `imu.csv` is extracted
  from the postprocessed solution.

These conventions are documented in the
[official data reference](https://github.com/utiasASRL/pyboreas/blob/8e100ff82ecd4992511da61b016398dbd230f205/DATA_REFERENCE.md).
The author confirms `imu.csv` angular rates in rad/s, acceleration in m/s²,
and gravity removed in [issue 24](https://github.com/utiasASRL/pyboreas/issues/24#issuecomment-1334042700).
Do not silently substitute it for raw accelerometer measurements.

## What the original raw logs contain

Raw GNSS receiver bytes are available in the sampled original recording; the
absence of `gps_raw.csv` does not mean raw GNSS was discarded. Inspecting the
first 1 MiB of the official
[`default1126_185009.000` recording](https://boreas.s3.amazonaws.com/boreas-2020-11-26-13-58/applanix/raw_logs/default1126_185009.000)
found Applanix `$GRP` binary packets with group IDs `10001`, `10006`, `4`, `1`,
`5`, `10`, `99`, `20000`, and `2`. This is a sample inspection, not a guarantee
that every sequence contains the same groups.

The vendor-authored
[POS LV interface specification](https://www.cds.caltech.edu/~murray/dgc05/upload/2/21/Applanix_Docs_PortInterface.pdf)
(PUBS-ICD-000036, revision 3, sections 3.2.2 and 3.3.2.1) defines group `10001`
as the primary GPS receiver's original byte stream. Packets can split receiver
messages; concatenating their payloads reconstructs the receiver stream, which
then needs receiver-specific decoding. The same specification identifies group
`1` as the vehicle navigation solution, group `2` as its performance metrics,
group `4` as time-tagged IMU data, and group `10006` as raw DMI data.

Consequently, two distinct exports could be developed: receiver GNSS decoded
from group `10001`, and real-time vehicle navigation decoded from group `1`.
The latter is already a GNSS/INS solution, not independent raw GNSS. Neither
should be confused with the separately supplied POSPac postprocessed ground
truth. A raw receiver `NavSatFix` exporter requires verification of the actual
receiver protocol, supported messages, time system, position datum, altitude
and quality fields; packet availability alone does not establish that mapping.
The [manufacturer describes POS LV](https://applanix.trimble.com/en/products/hardware/applanix-pos-lv)
as an integrated GNSS/INS system with both real-time navigation and raw data
logging.
