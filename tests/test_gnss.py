"""Geodetic height and uncertainty checks independent of ROS serialization."""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from boreas_to_ros2bag.data import gnss_rows


class HeightTest(unittest.TestCase):
    def test_recovers_absolute_height_from_original_boreas_enu(self):
        # Actual baseline position, independently matched to the SBET solution.
        with TemporaryDirectory() as directory:
            sequence = Path(directory)
            (sequence / 'applanix').mkdir()
            (sequence / 'applanix/gps_post_process.csv').write_text(
                'GPSTime,easting,northing,altitude,latitude,longitude\n'
                '1606417077.5213022,1.1272697327777532e-05,-1.217232784256339e-05,'
                '3.0016526579856873e-05,0.7641426285512647,-1.3869452891961105\n')
            sample = next(gnss_rows(sequence))
            self.assertEqual(sample[0], 1606417077521302200)
            self.assertAlmostEqual(sample[3], 153.67132212594151, places=7)


class UncertaintyTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.sequence = Path(self.directory.name)
        (self.sequence / 'applanix').mkdir()
        self.csv_path = self.sequence / 'applanix/gps_post_process.csv'
        self.csv_path.write_text(
            'GPSTime,easting,northing,altitude,latitude,longitude\n'
            '1606417077.5000001,0,0,0,0.7641426285531766,-1.3869452891985545\n'
            '1606417077.5050001,0,0,0,0.7641426285531766,-1.3869452891985545\n')
        self.sbet = np.zeros((2, 17))
        self.sbet[:, 0] = [413877.5000001, 413877.5050001]
        self.sbet[:, 1:4] = [0.7641426285531766, -1.3869452891985545, 153.67129211034626]
        self.sbet.tofile(self.sequence / 'applanix/sbet.out')
        self.errors = np.zeros((2, 10))
        self.errors[:, :4] = [[413877., .02, .03, .04], [413878., .04, .05, .06]]
        self.errors.tofile(self.sequence / 'applanix/smrmsg.out')

    def uncertainty(self):
        from boreas_to_ros2bag.data import GnssUncertainty
        return GnssUncertainty(self.sequence)

    def test_interpolates_reported_rms_then_squares_in_enu_order(self):
        uncertainty = self.uncertainty()
        self.assertEqual(uncertainty.metadata['matched_csv_samples'], 2)
        variances = uncertainty.diagonal(1606417077500000100)
        np.testing.assert_allclose(variances, [.040000002**2, .030000002**2, .050000002**2], atol=1e-12)

    def test_missing_auxiliary_file_keeps_uncertainty_unknown(self):
        (self.sequence / 'applanix/smrmsg.out').unlink()
        self.assertIsNone(self.uncertainty().diagonal(1606417077500000100))

    def test_rejects_guessed_gps_leap_second_shift(self):
        self.sbet[:, 0] += 18
        self.sbet.tofile(self.sequence / 'applanix/sbet.out')
        with self.assertRaisesRegex(ValueError, 'time alignment'):
            self.uncertainty()

    def test_rejects_mismatched_solution_and_truncated_error_file(self):
        self.sbet[:, 1] += 1e-5
        self.sbet.tofile(self.sequence / 'applanix/sbet.out')
        with self.assertRaisesRegex(ValueError, 'position'):
            self.uncertainty()
        (self.sequence / 'applanix/smrmsg.out').write_bytes(b'truncated')
        with self.assertRaisesRegex(ValueError, 'record'):
            self.uncertainty()

    def test_rejects_error_extrapolation_and_negative_uncertainty(self):
        self.errors[:, 0] -= 10
        self.errors.tofile(self.sequence / 'applanix/smrmsg.out')
        with self.assertRaisesRegex(ValueError, 'cover'):
            self.uncertainty()
        self.errors[:, 0] += 10
        self.errors[0, 1] = -1
        self.errors.tofile(self.sequence / 'applanix/smrmsg.out')
        with self.assertRaisesRegex(ValueError, 'uncertainty'):
            self.uncertainty()
