import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from boreas_to_ros2bag.data import (NS, Orientation, R_BASE_RAW, gnss_rows, lidar_points,
                                   matrix_quaternion, qmatrix, seconds_ns)


class ConventionsTest(unittest.TestCase):
    def test_decimal_timestamp_keeps_sub_microsecond_precision(self):
        self.assertEqual(seconds_ns('1606417077.5213022'),1606417077521302200)

    def test_stationary_raw_down_axis_aligns_specific_force_up(self):
        # Deliberately tilted IMU, not an identity-quaternion smoke test.
        acceleration = np.array([0.2,-0.3,-9.8])
        seed=(0,acceleration,np.zeros(3),np.linalg.norm(acceleration))
        estimator=Orientation(seed)
        q=estimator.update(0,np.zeros(3))
        np.testing.assert_allclose(qmatrix(q)@acceleration,[0,0,np.linalg.norm(acceleration)],atol=1e-10)
        world_base=qmatrix(q)@R_BASE_RAW.T
        self.assertAlmostEqual(np.arctan2(world_base[1,0],world_base[0,0]),0.)

    def test_gyro_integrates_body_rates_and_normalizes(self):
        estimator=Orientation((0,np.array([0.,0.,-9.8]),np.zeros(3),9.8))
        q0=estimator.update(0,np.array([0.,0.,1.]))
        for i in range(1,101):
            q=estimator.update(i*NS//100,np.array([0.,0.,1.]))
        expected=np.array([[np.cos(1),-np.sin(1),0],[np.sin(1),np.cos(1),0],[0,0,1]])
        np.testing.assert_allclose(qmatrix(q),qmatrix(q0)@expected,atol=1e-10)
        self.assertAlmostEqual(np.linalg.norm(q),1.)

    def test_mounting_half_turn_round_trip(self):
        np.testing.assert_allclose(qmatrix(matrix_quaternion(R_BASE_RAW)),R_BASE_RAW,atol=1e-12)

    def test_cloud_preserves_negative_time_intensity_and_ring(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)/'scan.bin'
            source=np.array([[1.,2.,3.,203.,127.,-0.05]],dtype='<f4')
            source.tofile(path)
            result=lidar_points(path)
            self.assertEqual(result.dtype['ring'],np.dtype('<u2'))
            self.assertEqual(result['ring'][0],127)
            self.assertEqual(result['intensity'][0],203.)
            self.assertEqual(result['time'][0],source[0,5])

    def test_malformed_cloud_is_rejected(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)/'scan.bin'
            path.write_bytes(b'bad')
            with self.assertRaises(ValueError):
                lidar_points(path)


class GnssTest(unittest.TestCase):
    def read_rows(self, text):
        with TemporaryDirectory() as directory:
            sequence = Path(directory)
            (sequence / 'applanix').mkdir()
            (sequence / 'applanix/gps_post_process.csv').write_text(text)
            return list(gnss_rows(sequence))

    def test_postprocessed_radians_and_exact_timestamp(self):
        rows = self.read_rows('GPSTime,latitude,longitude,easting,northing,altitude\n'
                              '1606417077.5213022,0.7641426285531766,-1.3869452891985545,0,0,0\n')
        self.assertEqual(rows[0][0], 1606417077521302200)
        self.assertAlmostEqual(rows[0][1], 43.78214756, places=7)
        self.assertAlmostEqual(rows[0][2], -79.46611149, places=7)
        self.assertAlmostEqual(rows[0][3], 153.67129211034626, places=7)

    def test_rejects_bad_coordinates_and_missing_columns(self):
        for latitude, longitude in [('nan', '0'), ('inf', '0'), ('2', '0'), ('0', '4')]:
            with self.subTest(latitude=latitude, longitude=longitude), self.assertRaises(ValueError):
                self.read_rows(f'GPSTime,latitude,longitude,easting,northing,altitude\n'
                               f'1,{latitude},{longitude},0,0,0\n')
        with self.assertRaisesRegex(ValueError, 'requires'):
            self.read_rows('GPSTime,easting,northing\n1,0,0\n')

    def test_rejects_duplicate_and_reversed_timestamps(self):
        for second in ['1', '0.5']:
            with self.subTest(second=second), self.assertRaisesRegex(ValueError, 'increase strictly'):
                self.read_rows('GPSTime,latitude,longitude,easting,northing,altitude\n'
                               f'1,0.7641426285531766,-1.3869452891985545,0,0,0\n'
                               f'{second},0.7641426285531766,-1.3869452891985545,0,0,0\n')


if __name__ == '__main__':
    unittest.main()
