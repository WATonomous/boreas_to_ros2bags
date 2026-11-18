"""Streaming conversion using Jazzy's standard messages and CDR serializer."""

import argparse
from collections import Counter
import hashlib
import heapq
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import rosbag2_py as bag
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from tf2_msgs.msg import TFMessage

from .data import (NS, Orientation, calibration, imu_rows, lidar_points,
                   matrix_quaternion, seconds_ns, sensor_files, stationary_seed)


def header(timestamp, frame):
    sec, nanosec = divmod(timestamp, NS)
    return Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=frame)


def assign_quaternion(target, q):
    target.x, target.y, target.z, target.w = map(float, q)


def static_message(timestamp, edges):
    transforms = []
    for parent, child, matrix in edges:
        msg = TransformStamped(header=header(timestamp, parent), child_frame_id=child)
        t = msg.transform.translation
        t.x, t.y, t.z = map(float, matrix[:3, 3])
        assign_quaternion(msg.transform.rotation, matrix_quaternion(matrix))
        transforms.append(msg)
    return TFMessage(transforms=transforms)


def cloud_stream(files, begin, end):
    fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
              for name, offset, kind in [('x',0,7), ('y',4,7), ('z',8,7),
                                         ('intensity',12,7), ('ring',16,4), ('time',20,7)]]
    for t, path in files:
        if not begin <= t < end:
            continue
        points = lidar_points(path)
        msg = PointCloud2(header=header(t, 'boreas_lidar'), height=1, width=len(points),
                          fields=fields, is_bigendian=False, point_step=24,
                          row_step=24*len(points), data=points.tobytes(), is_dense=True)
        yield t, 3, '/boreas/lidar/points', msg


def imu_stream(sequence, begin, end, orientation):
    for t, gyro, accel in imu_rows(sequence):
        if t >= end:
            break
        # Integrate earlier measurements as well for a later requested time window.
        q = orientation.update(t, gyro) if orientation else None
        if t < begin:
            continue
        msg = Imu(header=header(t, 'boreas_imu_raw'))
        msg.orientation.w = 1.
        msg.orientation_covariance[0] = -1.
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, accel)
        yield t, 1, '/boreas/imu/data_raw', msg
        if orientation:
            # Serialize the raw sample before adding the independently estimated orientation.
            if q is not None:
                assign_quaternion(msg.orientation, q)
                msg.orientation_covariance = [0.0025, 0., 0., 0., 0.0025, 0., 0., 0., 1.e6]
            yield t, 2, '/boreas/imu/data', msg


def camera_stream(sequence, files, begin, end):
    try:
        from PIL import Image as PillowImage
    except ImportError as error:
        raise RuntimeError('Camera export requires python3-pil (included in Dockerfile)') from error
    projection = np.loadtxt(sequence / 'calib/P_camera.txt')[:3, :4]
    for t, path in files:
        if not begin <= t < end:
            continue
        with PillowImage.open(path) as image:
            image = image.convert('RGB')
            width, height = image.size
            msg = Image(header=header(t, 'boreas_camera_optical'), height=height, width=width,
                        encoding='rgb8', is_bigendian=0, step=width*3, data=image.tobytes())
        info = CameraInfo(header=msg.header, height=height, width=width,
                          distortion_model='plumb_bob', d=[0.]*5,
                          k=projection[:3, :3].reshape(-1).tolist(),
                          r=np.eye(3).reshape(-1).tolist(), p=projection.reshape(-1).tolist())
        yield t, 4, '/boreas/camera/image_rect', msg
        yield t, 5, '/boreas/camera/camera_info', info


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='One Boreas sequence directory')
    parser.add_argument('--output', type=Path, required=True, help='New ROS 2 bag directory')
    parser.add_argument('--sensors', nargs='+', choices=['lidar','imu','camera'], default=['lidar','imu'])
    parser.add_argument('--start', default='0', help='Seconds from earliest selected sensor timestamp')
    parser.add_argument('--duration', help='Seconds to export; default all remaining data')
    parser.add_argument('--no-orientation', action='store_true', help='Export only raw IMU, not Eidos compatible')
    parser.add_argument('--seed-seconds', type=float, default=2., help='Initial stationary IMU interval')
    parser.add_argument('--stationary-gyro', type=float, default=0.02, help='Maximum seed gyro norm, rad/s')
    parser.add_argument('--raw-extrinsic', type=Path, help='Override 4x4 T_applanix_raw_imu')
    parser.add_argument('--max-bag-gib', type=float, default=4., help='Split MCAP files at this size')
    return parser


def convert(args):
    sequence, output = args.input.resolve(), args.output.resolve()
    if output == sequence or sequence in output.parents:
        raise ValueError('Output must be outside the raw sequence')
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    offset = seconds_ns(args.start)
    duration = seconds_ns(args.duration) if args.duration is not None else None
    if offset < 0 or (duration is not None and duration <= 0):
        raise ValueError('Require start >= 0 and duration > 0')
    if not 0 < args.seed_seconds <= 30 or args.stationary_gyro <= 0 or args.max_bag_gib < 0.01:
        raise ValueError('Invalid initialization or file size limit')
    files = {}
    first_stamps = []
    for sensor, suffix in [('lidar','.bin'), ('camera','.png')]:
        if sensor in args.sensors:
            files[sensor] = sensor_files(sequence, sensor, suffix)
            first_stamps.append(files[sensor][0][0])
    if 'imu' in args.sensors:
        first_stamps.append(next(imu_rows(sequence))[0])
    begin = min(first_stamps) + offset
    end = begin + duration if duration else 2**63-1
    edges = calibration(sequence, 'camera' in args.sensors, args.raw_extrinsic)
    orientation = None
    if 'imu' in args.sensors and not args.no_orientation:
        seed = stationary_seed(sequence, args.seed_seconds, args.stationary_gyro)
        r_base_raw = edges[0][2][:3,:3] @ edges[1][2][:3,:3]
        orientation = Orientation(seed, r_base_raw)
    streams = []
    if 'imu' in args.sensors:
        streams.append(imu_stream(sequence, begin, end, orientation))
    if 'lidar' in args.sensors:
        streams.append(cloud_stream(files['lidar'], begin, end))
    if 'camera' in args.sensors:
        streams.append(camera_stream(sequence, files['camera'], begin, end))
    events = heapq.merge(*streams, key=lambda item: item[:2])
    first = next(events, None)
    if first is None:
        raise ValueError('Selected time interval contains no sensor data')
    output.parent.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(output.parent).free < 512*1024**2:
        raise ValueError('Need at least 512 MiB free in output filesystem')
    config = Path(__file__).resolve().parents[1] / 'config/mcap.yaml'
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(output), storage_id='mcap',
                                  max_bagfile_size=int(args.max_bag_gib*1024**3),
                                  max_cache_size=0, storage_config_uri=str(config)),
                bag.ConverterOptions('cdr', 'cdr'))
    counts = Counter()
    topics = {}
    last = first[0]
    try:
        static_qos = bag._storage.QoS(1).reliable().transient_local()
        sensor_qos = bag._storage.QoS(10).reliable().durability_volatile()

        def write(t, topic, msg):
            nonlocal last
            if t < last:
                raise ValueError('Internal error: out-of-order message')
            last = t
            if topic not in topics:
                cls = type(msg)
                type_name = f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}'
                writer.create_topic(bag.TopicMetadata(id=len(topics)+1, name=topic,
                                    type=type_name, serialization_format='cdr',
                                    offered_qos_profiles=[static_qos if topic == '/tf_static' else sensor_qos]))
                topics[topic] = type_name
            writer.write(topic, serialize_message(msg), t)
            counts[topic] += 1

        write(first[0], '/tf_static', static_message(first[0], edges))
        write(first[0], first[2], first[3])
        for t, _, topic, msg in events:
            write(t, topic, msg)
            if topic == '/boreas/lidar/points' and counts[topic] % 100 == 0:
                print(f'{counts[topic]} clouds; {(t-begin)/NS:.1f}s', flush=True)
                if shutil.disk_usage(output).free < 256*1024**2:
                    raise OSError('Output filesystem nearly full; conversion stopped')
    finally:
        writer.close()
    report = {
        'input': str(sequence), 'output': str(output), 'sensors': args.sensors,
        'requested_start_ns': begin, 'requested_end_ns': end if duration else None,
        'first_ns': first[0], 'last_ns': last, 'counts': dict(counts), 'topics': topics,
        'timestamp_policy': 'Dataset UTC epochs preserved; decimal seconds/filename microseconds to ns; no leap offset',
        'orientation': None if orientation is None else {
            'method': 'stationary gravity tilt + gyro propagation; no absolute heading or GT',
            'seed_end_ns': orientation.ready_ns, 'gyro_bias_rad_s': orientation.bias.tolist(),
            'measured_gravity_m_s2': orientation.gravity,
        },
        'calibration': [{'parent': p, 'child': c, 'T_parent_child': m.tolist()} for p,c,m in edges],
        'calibration_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in sorted((sequence/'calib').glob('*.txt'))},
        'raw_imu_origin': 'ASSUMPTION: no separate raw-IMU lever arm is published; axes-only default '
                          'at Applanix reference origin. Override with --raw-extrinsic if measured.',
        'ground_truth_used': False,
    }
    (output / 'conversion.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return report


def main():
    try:
        convert(make_parser().parse_args())
    except (ValueError, OSError, RuntimeError, StopIteration) as error:
        print(f'Conversion failed: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
