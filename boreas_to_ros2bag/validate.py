"""Read every record through Jazzy, deserialize CDR and check dataset/TF invariants."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import sys

import numpy as np
import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from .data import NS, POINT_DTYPE, GnssUncertainty, gnss_rows, imu_rows, qmatrix, sensor_files


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(path, compare_source=False):
    if not path.is_absolute() and os.environ.get('BOREAS_BAG_ROOT'):
        path = Path(os.environ['BOREAS_BAG_ROOT']) / path
    report = json.loads((path/'conversion.json').read_text())
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(path), storage_id='mcap'), bag.ConverterOptions('cdr','cdr'))
    metadata = reader.get_metadata()
    topics = {t.name: t for t in reader.get_all_topics_and_types()}
    require(all(t.serialization_format == 'cdr' for t in topics.values()), 'Non-CDR topic')
    qos = bag._storage.convert_rclcpp_qos_to_rclpy_qos(topics['/tf_static'].offered_qos_profiles[0])
    require(qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
            and qos.reliability == ReliabilityPolicy.RELIABLE, 'Wrong static TF QoS')
    definitions = reader.get_all_message_definitions()
    require({d.topic_type for d in definitions} == {t.type for t in topics.values()}, 'Missing message schemas')
    require(all(d.encoding in ('ros2msg','ros2idl') and d.encoded_message_definition for d in definitions),
            'Missing embedded ROS 2 schema definitions')
    require('/clock' not in topics and not any('ground_truth' in t for t in topics), 'Unexpected estimator input')
    classes = {name: get_message(t.type) for name,t in topics.items()}
    sequence = Path(report['input'])
    begin = report['requested_start_ns']
    end = report['requested_end_ns'] or 2**63-1
    source_imu = (row for row in imu_rows(sequence) if begin <= row[0] < end) if compare_source and 'imu' in report['sensors'] else None
    source_gnss = (row for row in gnss_rows(sequence) if begin <= row[0] < end) if compare_source and 'gnss' in report['sensors'] else None
    gnss_policy = report.get('gnss') or {}
    height_available = gnss_policy.get('altitude_method') == 'boreas_enu_to_wgs84'
    has_uncertainty = gnss_policy.get('uncertainty', {}).get('available', False)
    source_uncertainty = GnssUncertainty(sequence) if compare_source and has_uncertainty else None
    if source_uncertainty is not None:
        require(source_uncertainty.metadata['available'], 'GNSS uncertainty source files are missing')
    require(report['ground_truth_used'] == ('gnss' in report['sensors']), 'GNSS provenance mismatch')
    counts = Counter()
    first = None
    last = -1
    valid_stationary = longest_stationary = invalid_orientations = 0
    cloud_ranges = None
    tf_edges = {}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        require(timestamp >= last, 'Bag records are not chronological')
        last = timestamp
        first = timestamp if first is None else first
        msg = deserialize_message(data, classes[topic])
        counts[topic] += 1
        if topic == '/tf_static':
            for tf in msg.transforms:
                parent, child = tf.header.frame_id, tf.child_frame_id
                require(child not in tf_edges, f'Duplicate static parent: {child}')
                q = np.array([tf.transform.rotation.x, tf.transform.rotation.y,
                              tf.transform.rotation.z, tf.transform.rotation.w])
                require(abs(np.linalg.norm(q)-1) < 1e-9, 'Non-unit TF quaternion')
                matrix = np.eye(4)
                matrix[:3,:3] = qmatrix(q)
                matrix[:3,3] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
                tf_edges[child] = parent, matrix
            continue
        require(msg.header.stamp.sec*NS + msg.header.stamp.nanosec == timestamp, 'Header/record timestamp mismatch')
        require(msg.header.frame_id in tf_edges, f'Unconnected sensor frame {msg.header.frame_id}')
        if topic == '/boreas/lidar/points':
            fields = {f.name: (f.offset,f.datatype) for f in msg.fields}
            require(fields == {'x':(0,7),'y':(4,7),'z':(8,7),'intensity':(12,7),'ring':(16,4),'time':(20,7)}, 'Wrong cloud layout')
            require(msg.point_step == 24 and len(msg.data) == msg.row_step == msg.width*24 and msg.height == 1,
                    'Cloud size mismatch')
            points = np.frombuffer(msg.data, dtype=POINT_DTYPE)
            require(np.isfinite(points['x']).all() and points['ring'].max() <= 127, 'Invalid cloud')
            if compare_source:
                source = np.fromfile(sequence/'lidar'/f'{timestamp//1000}.bin', dtype='<f4').reshape(-1,6)
                require(len(source) == len(points), 'Dropped source points')
                for i,name in enumerate(POINT_DTYPE.names):
                    require(np.array_equal(points[name], source[:,i]), f'Changed source cloud field {name}')
            cloud_ranges = {'points_in_last_scan': len(points), 'ring_min': int(points['ring'].min()),
                            'ring_max': int(points['ring'].max()),
                            'relative_time_min_s': float(points['time'].min()),
                            'relative_time_max_s': float(points['time'].max())}
        elif topic.startswith('/boreas/imu/'):
            gyro = np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
            accel = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
            require(np.isfinite(gyro).all() and np.isfinite(accel).all(), 'Nonfinite IMU')
            if topic.endswith('data_raw'):
                require(msg.orientation_covariance[0] == -1., 'Raw orientation must be unavailable')
                if source_imu is not None:
                    t,g,a = next(source_imu)
                    require(t == timestamp and np.array_equal(g,gyro) and np.array_equal(a,accel), 'Changed raw IMU')
            else:
                valid = msg.orientation_covariance[0] >= 0
                if valid:
                    q = np.array([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
                    require(abs(np.linalg.norm(q)-1) < 1e-9, 'Invalid estimated orientation')
                    valid_stationary = valid_stationary+1 if np.linalg.norm(gyro) < 0.02 else 0
                    longest_stationary = max(longest_stationary, valid_stationary)
                else:
                    invalid_orientations += 1
        elif topic == '/boreas/gnss/postprocessed/fix':
            require(msg.header.frame_id == 'boreas_applanix', 'Wrong GNSS reference frame')
            require(np.isfinite([msg.latitude, msg.longitude]).all()
                    and -90 <= msg.latitude <= 90 and -180 <= msg.longitude <= 180, 'Invalid GNSS coordinates')
            require(np.isfinite(msg.altitude) if height_available else np.isnan(msg.altitude),
                    'GNSS altitude disagrees with its recorded policy')
            require(msg.status.status == gnss_policy.get('status_value', -2)
                    and msg.status.service == 0, 'GNSS status policy mismatch')
            if has_uncertainty:
                require(msg.position_covariance_type == msg.COVARIANCE_TYPE_APPROXIMATED
                        and np.isfinite(msg.position_covariance).all()
                        and all(msg.position_covariance[i] >= 0 for i in (0, 4, 8))
                        and all(msg.position_covariance[i] == 0 for i in (1, 2, 3, 5, 6, 7)),
                        'Invalid interpolated GNSS covariance')
                if source_uncertainty is not None:
                    require(np.allclose([msg.position_covariance[i] for i in (0, 4, 8)],
                                        source_uncertainty.diagonal(timestamp), rtol=1e-12, atol=1e-15),
                            'GNSS covariance differs from source SMRMSG')
            else:
                require(msg.position_covariance_type == msg.COVARIANCE_TYPE_UNKNOWN
                        and all(x == 0 for x in msg.position_covariance), 'GNSS covariance must be unknown')
            if source_gnss is not None:
                t, latitude, longitude, altitude = next(source_gnss)
                require(t == timestamp and msg.latitude == latitude and msg.longitude == longitude,
                        'Changed source GNSS coordinates or timestamp')
                if height_available:
                    require(abs(msg.altitude-altitude) < 1e-6, 'GNSS ellipsoid height mismatch')
        elif topic.endswith('image_rect'):
            require(msg.encoding == 'rgb8' and len(msg.data) == msg.width*msg.height*3, 'Invalid camera image')
        elif topic.endswith('camera_info'):
            require(msg.distortion_model == 'plumb_bob' and all(x == 0 for x in msg.d), 'Rectified camera distortion')
            if compare_source:
                p = np.loadtxt(sequence/'calib/P_camera.txt')[:3,:4]
                require(np.allclose(msg.p,p.reshape(-1)), 'Camera projection mismatch')
    require(dict(counts) == report['counts'], 'Conversion count mismatch')
    require(counts == Counter({x.topic_metadata.name:x.message_count for x in metadata.topics_with_message_count}),
            'Bag metadata count mismatch')
    require(first == report['first_ns'] and last == report['last_ns'], 'Bag time range mismatch')
    require(counts['/tf_static'] == 1, 'Expected one aggregate static TF message')
    expected_edges = {e['child']:(e['parent'],np.array(e['T_parent_child'])) for e in report['calibration']}
    require(set(tf_edges) == set(expected_edges), 'TF tree edge mismatch')
    for child,(parent,matrix) in tf_edges.items():
        expected_parent, expected_matrix = expected_edges[child]
        require(parent == expected_parent and np.allclose(matrix,expected_matrix,atol=1e-6), f'TF mismatch {child}')
        seen = {child}
        while parent in tf_edges:
            require(parent not in seen, 'TF cycle')
            seen.add(parent)
            parent = tf_edges[parent][0]
        require(parent == 'boreas_base', 'Disconnected static TF tree')
    if compare_source:
        require(np.allclose(tf_edges['boreas_lidar'][1],
                            np.loadtxt(sequence/'calib/T_applanix_lidar.txt'),atol=1e-6),
                'TF differs from source lidar extrinsic')
        if 'camera' in report['sensors']:
            require(np.allclose(tf_edges['boreas_camera_optical'][1],
                                np.linalg.inv(np.loadtxt(sequence/'calib/T_camera_lidar.txt')),atol=1e-6),
                    'TF differs from inverse camera extrinsic')
        for sensor,suffix,topic in [('lidar','.bin','/boreas/lidar/points'), ('camera','.png','/boreas/camera/image_rect')]:
            if sensor in report['sensors']:
                expected = sum(begin <= t < end for t,p in sensor_files(sequence,sensor,suffix))
                require(counts[topic] == expected, f'Missing {sensor} files in interval')
        if source_imu is not None:
            require(next(source_imu,None) is None, 'Missing source IMU samples')
        if source_gnss is not None:
            require(next(source_gnss,None) is None, 'Missing source GNSS samples')
    reader.close()
    result = {'verified': True, 'source_compared': compare_source, 'counts': dict(counts),
              'first_ns':first,'last_ns':last,'duration_s':(last-first)/NS,
              'schemas': {d.topic_type:d.encoding for d in definitions},
              'longest_liso_stationary_orientation_run':longest_stationary,
              'estimated_orientation_unavailable_samples':invalid_orientations,
              'static_tf': {c:p for c,(p,m) in tf_edges.items()}, 'last_cloud':cloud_ranges}
    (path/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--compare-source',action='store_true')
    args=parser.parse_args()
    try:
        validate(args.bag,args.compare_source)
    except (ValueError,OSError,RuntimeError,StopIteration) as error:
        print(f'Validation failed: {error}',file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
