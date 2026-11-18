"""Dataset conventions, independent of ROS and post-processed pose files."""

import csv
from decimal import Decimal
from pathlib import Path

import numpy as np

NS = 1_000_000_000
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
