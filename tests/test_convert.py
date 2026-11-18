"""Native Jazzy MCAP round trips, including opt-in GNSS and source comparison."""

import contextlib
import io
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

if os.environ.get('ROS_DISTRO') != 'jazzy':
    raise unittest.SkipTest('Native MCAP integration tests require ROS 2 Jazzy')

from boreas_to_ros2bag.convert import convert, make_parser
from boreas_to_ros2bag.validate import validate
import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix


class ConversionTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.sequence = self.root / 'sequence'
        for folder in ['applanix', 'calib', 'lidar']:
            (self.sequence / folder).mkdir(parents=True)
        np.savetxt(self.sequence / 'calib/T_applanix_lidar.txt', np.eye(4))
        (self.sequence / 'applanix/gps_post_process.csv').write_text(
            'GPSTime,latitude,longitude,easting,northing,altitude\n'
            '1606417077.5213022,0.7641426285531766,-1.3869452891985545,0,0,0\n'
            '1606417077.5263022,0.7641426285531766,-1.3869452891985545,0,0,0.00001\n'
            '1606417077.5313022,0.7641426285531766,-1.3869452891985545,0,0,0.00002\n')
        (self.sequence / 'applanix/imu_raw.csv').write_text(
            'GPSTime,angvel_x,angvel_y,angvel_z,accelx,accely,accelz\n'
            '1606417077.5163022,0,0,0,0,0,-9.8\n'
            '1606417077.5263022,0,0,0,0,0,-9.8\n')
        np.array([[1., 2., 3., 40., 127., -0.05]], dtype='<f4').tofile(
            self.sequence / 'lidar/1606417077521303.bin')

    def round_trip(self, *options):
        output = self.root / 'bag'
        args = make_parser().parse_args(['--input', str(self.sequence), '--output', str(output), *options])
        with contextlib.redirect_stdout(io.StringIO()):
            report = convert(args)
            result = validate(output, compare_source=True)
        self.assertTrue(result['verified'])
        self.assertTrue(result['source_compared'])
        return report, result

    def first_gnss(self):
        reader = bag.SequentialReader()
        reader.open(bag.StorageOptions(uri=str(self.root/'bag'), storage_id='mcap'),
                    bag.ConverterOptions('cdr', 'cdr'))
        reader.set_filter(bag.StorageFilter(topics=['/boreas/gnss/postprocessed/fix']))
        _, data, timestamp = reader.read_next()
        msg = deserialize_message(data, NavSatFix)
        reader.close()
        return msg, timestamp

    def test_gnss_only_window_preserves_boundaries_and_provenance(self):
        report, result = self.round_trip('--sensors', 'gnss', '--start', '0.005', '--duration', '0.005')
        self.assertEqual(result['counts'], {'/tf_static': 1, '/boreas/gnss/postprocessed/fix': 1})
        self.assertEqual(result['first_ns'], 1606417077526302200)
        self.assertEqual(result['last_ns'], result['first_ns'])
        self.assertTrue(report['ground_truth_used'])
        self.assertEqual(report['gnss']['source'], 'applanix/gps_post_process.csv')
        self.assertIn('sensor_msgs/msg/NavSatFix', result['schemas'])
        msg, timestamp = self.first_gnss()
        self.assertEqual(timestamp, 1606417077526302200)
        self.assertAlmostEqual(msg.altitude, 153.67130211034626, places=7)
        self.assertEqual(msg.status.status, 0)
        self.assertEqual(msg.status.service, 0)
        self.assertEqual(msg.position_covariance_type, NavSatFix.COVARIANCE_TYPE_UNKNOWN)

    def test_aligned_uncertainty_round_trips_as_enu_variances(self):
        sbet = np.zeros((3, 17))
        sbet[:, 0] = [413877.5213022, 413877.5263022, 413877.5313022]
        sbet[:, 1:3] = [0.7641426285531766, -1.3869452891985545]
        sbet[:, 3] = [153.67129211034626, 153.67130211034626, 153.67131211034626]
        sbet.tofile(self.sequence/'applanix/sbet.out')
        errors = np.zeros((2, 10))
        errors[:, :4] = [[413877., .02, .03, .04], [413878., .04, .05, .06]]
        errors.tofile(self.sequence/'applanix/smrmsg.out')
        report, _ = self.round_trip('--sensors', 'gnss')
        self.assertEqual(report['gnss']['uncertainty']['matched_csv_samples'], 3)
        msg, _ = self.first_gnss()
        self.assertEqual(msg.position_covariance_type, NavSatFix.COVARIANCE_TYPE_APPROXIMATED)
        np.testing.assert_allclose([msg.position_covariance[i] for i in (0, 4, 8)],
                                   [(.03+.02*.5213022)**2, (.02+.02*.5213022)**2,
                                    (.04+.02*.5213022)**2], atol=1e-12)

    def test_unknown_status_remains_available_as_an_explicit_option(self):
        self.round_trip('--sensors', 'gnss', '--gnss-status', 'unknown')
        msg, _ = self.first_gnss()
        self.assertEqual(msg.status.status, -2)
        self.assertTrue(np.isfinite(msg.altitude))

    def test_gnss_merges_with_lidar_and_raw_imu(self):
        report, result = self.round_trip('--sensors', 'lidar', 'imu', 'gnss', '--no-orientation')
        self.assertEqual(result['counts'], {'/tf_static': 1, '/boreas/imu/data_raw': 2,
                                           '/boreas/lidar/points': 1,
                                           '/boreas/gnss/postprocessed/fix': 3})
        self.assertTrue(report['ground_truth_used'])

    def test_default_selection_does_not_require_or_export_gnss(self):
        (self.sequence / 'applanix/gps_post_process.csv').unlink()
        report, result = self.round_trip('--no-orientation')
        self.assertEqual(result['counts'], {'/tf_static': 1, '/boreas/imu/data_raw': 2,
                                           '/boreas/lidar/points': 1})
        self.assertFalse(report['ground_truth_used'])
        self.assertIsNone(report['gnss'])
