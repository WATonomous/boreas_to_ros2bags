"""Dataset conventions and streaming readers, independent of ROS."""

import csv
from decimal import Decimal
from pathlib import Path

import numpy as np

NS = 1_000_000_000
# Shared origin published by the Boreas maintainers, radians/radians/metres.
BOREAS_ORIGIN = (0.7641426285531766, -1.3869452891985545, 153.67129211034626)
WGS84_A = 6378137.
WGS84_E2 = 1 - (1 - 1/298.257223563)**2


def geodetic_surface(latitude, longitude):
    n = WGS84_A / np.sqrt(1 - WGS84_E2*np.sin(latitude)**2)
    return np.array([n*np.cos(latitude)*np.cos(longitude),
                     n*np.cos(latitude)*np.sin(longitude), n*(1-WGS84_E2)*np.sin(latitude)])


def geodetic_normal(latitude, longitude):
    return np.array([np.cos(latitude)*np.cos(longitude),
                     np.cos(latitude)*np.sin(longitude), np.sin(latitude)])


_lat0, _lon0, _height0 = BOREAS_ORIGIN
ECEF_ORIGIN = geodetic_surface(_lat0, _lon0) + _height0*geodetic_normal(_lat0, _lon0)
R_ECEF_ENU = np.array([
    [-np.sin(_lon0), -np.sin(_lat0)*np.cos(_lon0), np.cos(_lat0)*np.cos(_lon0)],
    [np.cos(_lon0), -np.sin(_lat0)*np.sin(_lon0), np.cos(_lat0)*np.sin(_lon0)],
    [0., np.cos(_lat0), np.sin(_lat0)],
])
R_BASE_APPLANIX = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
R_APPLANIX_RAW = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
R_BASE_RAW = R_BASE_APPLANIX @ R_APPLANIX_RAW
POINT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24,
})


def seconds_ns(value):
    """Parse decimal seconds without first rounding an epoch through float."""
    return int((Decimal(str(value)) * NS).to_integral_value())


def sensor_files(sequence, sensor, suffix):
    files = sorted((sequence / sensor).glob('*' + suffix))
    if not files:
        raise ValueError(f'No {sensor} files in {sequence}')
    return [(int(p.stem) * 1000, p) for p in files]


def imu_rows(sequence):
    previous = -1
    with (sequence / 'applanix/imu_raw.csv').open() as stream:
        for row in csv.DictReader(stream):
            t = seconds_ns(row['GPSTime'])
            if t <= previous:
                raise ValueError('Raw IMU timestamps must increase strictly')
            previous = t
            gyro = np.array([float(row['angvel_' + k]) for k in 'xyz'])
            accel = np.array([float(row['accel' + k]) for k in 'xyz'])
            if not np.isfinite(gyro).all() or not np.isfinite(accel).all():
                raise ValueError(f'Nonfinite IMU measurement at {t}')
            yield t, gyro, accel


def gnss_rows(sequence):
    """Read post-processed geodetic positions, not raw receiver fixes.

    Convert radians to degrees and recover ellipsoid height from the shared
    ENU origin. Check that local and geodetic positions describe the same point.
    """
    previous = -1
    with (sequence / 'applanix/gps_post_process.csv').open() as stream:
        rows = csv.DictReader(stream)
        required = {'GPSTime', 'latitude', 'longitude', 'easting', 'northing', 'altitude'}
        if not required.issubset(rows.fieldnames or []):
            raise ValueError('gps_post_process.csv requires GPSTime, latitude, longitude, '
                             'easting, northing, altitude columns')
        for row in rows:
            t = seconds_ns(row['GPSTime'])
            if t <= previous:
                raise ValueError('GNSS timestamps must increase strictly')
            previous = t
            latitude, longitude = float(row['latitude']), float(row['longitude'])
            if (not np.isfinite([latitude, longitude]).all()
                    or not -np.pi/2 <= latitude <= np.pi/2
                    or not -np.pi <= longitude <= np.pi):
                raise ValueError(f'Invalid GNSS latitude/longitude at {t}; expected radians')
            enu = np.array([float(row[k]) for k in ('easting', 'northing', 'altitude')])
            if not np.isfinite(enu).all():
                raise ValueError(f'Nonfinite GNSS ENU position at {t}')
            normal = geodetic_normal(latitude, longitude)
            delta = ECEF_ORIGIN + R_ECEF_ENU @ enu - geodetic_surface(latitude, longitude)
            height = float(delta @ normal)
            if np.linalg.norm(delta - height*normal) > 0.01:
                raise ValueError(f'GNSS ENU/geodetic position mismatch at {t}; '
                                 'expected the published original Boreas origin')
            yield t, float(np.degrees(latitude)), float(np.degrees(longitude)), height


def utc_week_seconds(timestamp):
    """Sunday-based UTC week time without rounding a Unix epoch through float."""
    day = timestamp // (86400*NS)
    sunday = day - (day+4) % 7  # Unix epoch was a Thursday.
    return (timestamp - sunday*86400*NS) / NS


def binary_records(path, columns):
    if not path.stat().st_size or path.stat().st_size % (columns*8):
        raise ValueError(f'{path}: expected complete {columns}-double records')
    return np.memmap(path, dtype='<f8', mode='r').reshape(-1, columns)


class GnssUncertainty:
    """Use SMRMSG RMS only after verifying the CSV/SBET time correspondence.

    SBET permits different time systems. No leap-second shift is guessed: every
    CSV position must match a SBET position at the same UTC second of week.
    """

    def __init__(self, sequence):
        paths = [sequence/'applanix'/name for name in ('sbet.out', 'smrmsg.out')]
        self.errors = None
        self.metadata = {'available': False, 'missing_files': [p.name for p in paths if not p.is_file()]}
        if self.metadata['missing_files']:
            return
        sbet = binary_records(paths[0], 17)
        errors = binary_records(paths[1], 10)
        if (len(sbet) < 2 or not np.isfinite(sbet[:, :4]).all()
                or np.any(np.diff(sbet[:, 0]) <= 0)):
            raise ValueError('Invalid SBET positions/time alignment reference')
        if (len(errors) < 2 or not np.isfinite(errors[:, :4]).all()
                or np.any(errors[:, 1:4] < 0) or np.any(np.diff(errors[:, 0]) <= 0)
                or np.any(np.diff(errors[:, 0]) > 2)):
            raise ValueError('Invalid SMRMSG position uncertainty or time spacing')
        count, previous = 0, -1
        sbet_times = np.array(sbet[:, 0])
        max_dt = max_position_error = 0.
        for timestamp, latitude, longitude, _ in gnss_rows(sequence):
            tow = utc_week_seconds(timestamp)
            upper = int(np.searchsorted(sbet_times, tow))
            choices = [i for i in (upper-1, upper) if 0 <= i < len(sbet)]
            index = min(choices, key=lambda i: abs(sbet[i, 0]-tow))
            dt = abs(sbet[index, 0]-tow)
            if dt > 1e-6 or index <= previous:
                raise ValueError('GNSS CSV/SBET time alignment failed; refusing to guess a GPS/UTC offset')
            error = float(np.max(np.abs(sbet[index, 1:3] - np.radians([latitude, longitude]))))
            if error > 1e-10:
                raise ValueError('GNSS CSV/SBET position mismatch; cannot align uncertainty')
            if not errors[0, 0] <= tow <= errors[-1, 0]:
                raise ValueError('SMRMSG does not cover the GNSS timestamps; refusing extrapolation')
            previous = index
            count += 1
            max_dt, max_position_error = max(max_dt, dt), max(max_position_error, error)
        if not count:
            raise ValueError('No GNSS positions to align with SMRMSG')
        self.errors = errors
        self.metadata = {
            'available': True, 'source': 'applanix/smrmsg.out',
            'alignment_reference': 'applanix/sbet.out', 'matched_csv_samples': count,
            'time_policy': 'UTC seconds-of-week, checked against every CSV/SBET position; no guessed leap offset',
            'max_time_residual_s': float(max_dt), 'max_position_residual_rad': max_position_error,
            'method': 'Linearly interpolate published N/E/D RMS in metres, then square and reorder to ENU',
            'covariance_type': 'APPROXIMATED; interpolated diagonal only, no published cross terms',
        }

    def diagonal(self, timestamp):
        if self.errors is None:
            return None
        tow = utc_week_seconds(timestamp)
        if not self.errors[0, 0] <= tow <= self.errors[-1, 0]:
            raise ValueError('SMRMSG does not cover GNSS timestamp')
        return [float(np.interp(tow, self.errors[:, 0], self.errors[:, i]))**2 for i in (2, 1, 3)]


def lidar_points(path):
    if path.stat().st_size % 24:
        raise ValueError(f'{path}: expected six float32 values per point')
    raw = np.fromfile(path, dtype='<f4').reshape(-1, 6)
    if not len(raw) or not np.isfinite(raw).all():
        raise ValueError(f'{path}: empty or nonfinite cloud')
    rings = raw[:, 4]
    if np.any(rings != np.floor(rings)) or np.any((rings < 0) | (rings > 127)):
        raise ValueError(f'{path}: invalid Alpha Prime ring IDs')
    if np.max(np.abs(raw[:, 5])) > 0.2:
        raise ValueError(f'{path}: unexpected relative point times')
    points = np.zeros(len(raw), dtype=POINT_DTYPE)
    for i, name in enumerate(points.dtype.names):
        points[name] = raw[:, i]
    return points


def transform(path):
    matrix = np.loadtxt(path)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-5)):
        raise ValueError(f'{path}: expected rigid T_parent_child')
    return matrix


def calibration(sequence, include_camera=False, raw_extrinsic=None):
    base_applanix = np.eye(4)
    base_applanix[:3, :3] = R_BASE_APPLANIX
    applanix_raw = np.eye(4)
    applanix_raw[:3, :3] = R_APPLANIX_RAW
    if raw_extrinsic:
        applanix_raw = transform(raw_extrinsic)
    edges = [
        ('boreas_base', 'boreas_applanix', base_applanix),
        ('boreas_applanix', 'boreas_imu_raw', applanix_raw),
        ('boreas_applanix', 'boreas_lidar', transform(sequence / 'calib/T_applanix_lidar.txt')),
    ]
    if include_camera:
        edges.append(('boreas_lidar', 'boreas_camera_optical',
                      np.linalg.inv(transform(sequence / 'calib/T_camera_lidar.txt'))))
    return edges


def qmultiply(a, b):
    av, aw, bv, bw = a[:3], a[3], b[:3], b[3]
    return np.r_[aw * bv + bw * av + np.cross(av, bv), aw * bw - av @ bv]


def qmatrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def matrix_quaternion(matrix):
    # Stable also for the raw IMU's near-180-degree mounting rotation.
    r = matrix[:3, :3]
    candidates = np.array([1+r[0,0]-r[1,1]-r[2,2],
                           1-r[0,0]+r[1,1]-r[2,2],
                           1-r[0,0]-r[1,1]+r[2,2], 1+np.trace(r)])
    i = int(np.argmax(candidates))
    s = 2 * np.sqrt(max(candidates[i], 0))
    if i == 3:
        q = np.array([(r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s,
                      (r[1,0]-r[0,1])/s, s/4])
    else:
        j, k = (i+1) % 3, (i+2) % 3
        q = np.zeros(4)
        q[i], q[j], q[k], q[3] = s/4, (r[j,i]+r[i,j])/s, (r[k,i]+r[i,k])/s, (r[k,j]-r[j,k])/s
    return q / np.linalg.norm(q)


def stationary_seed(sequence, seconds=2., gyro_limit=0.02):
    """Require an initial stationary interval; never read post-processed poses."""
    rows = []
    for row in imu_rows(sequence):
        rows.append(row)
        if row[0] - rows[0][0] >= seconds_ns(seconds):
            break
    if len(rows) < 2 or rows[-1][0] - rows[0][0] < seconds_ns(seconds):
        raise ValueError('Not enough raw IMU data to initialize tilt')
    gyros = np.array([r[1] for r in rows])
    accels = np.array([r[2] for r in rows])
    gravity = float(np.linalg.norm(accels.mean(axis=0)))
    if (np.max(np.linalg.norm(gyros, axis=1)) >= gyro_limit
            or not 9.3 < gravity < 10.3 or np.max(accels.std(axis=0)) > 0.35):
        raise ValueError('Initial raw IMU interval is not stationary; use --no-orientation '
                         'for raw-only export, or provide a dataset with stationary initialization')
    return rows[-1][0], accels.mean(axis=0), gyros.mean(axis=0), gravity


class Orientation:
    """Gravity tilt seed followed by bias-corrected gyro integration, free yaw."""

    def __init__(self, seed, r_base_raw=R_BASE_RAW):
        self.ready_ns, acceleration, self.bias, self.gravity = seed
        ax, ay, az = r_base_raw @ acceleration
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.hypot(ay, az))
        qr = np.array([np.sin(roll/2), 0., 0., np.cos(roll/2)])
        qp = np.array([0., np.sin(pitch/2), 0., np.cos(pitch/2)])
        self.q = qmultiply(qmultiply(qp, qr), matrix_quaternion(r_base_raw))
        self.previous_t = self.ready_ns
        self.previous_gyro = None

    def update(self, timestamp, gyro):
        if timestamp < self.ready_ns:
            return None
        dt = (timestamp - self.previous_t) / NS
        if dt > 0.1:
            raise ValueError(f'IMU gap {dt:.3f}s; gyro orientation propagation is invalid')
        if self.previous_gyro is not None:
            step = (0.5 * (gyro + self.previous_gyro) - self.bias) * dt
            angle = np.linalg.norm(step)
            dq = np.r_[step * (np.sin(angle/2)/angle if angle > 1e-12 else 0.5), np.cos(angle/2)]
            self.q = qmultiply(self.q, dq)
            self.q /= np.linalg.norm(self.q)
        self.previous_t, self.previous_gyro = timestamp, gyro.copy()
        return self.q.copy()
