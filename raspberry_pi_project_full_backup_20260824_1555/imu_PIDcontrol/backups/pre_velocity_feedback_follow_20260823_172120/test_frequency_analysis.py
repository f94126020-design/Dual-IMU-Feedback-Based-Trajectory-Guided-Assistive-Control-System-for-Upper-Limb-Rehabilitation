import math
import tempfile
import types
import unittest

import numpy as np

from frequency_analysis import FrequencyIdentificationSession


class FrequencyAnalysisTests(unittest.TestCase):
    def test_chirp_is_faded_and_bounded(self):
        session = FrequencyIdentificationSession()
        session.start({"signal_type": "chirp", "duration": 20, "amplitude": 12,
                       "f_start": 0.1, "f_end": 2.0}, initial_angle=20, now=100)
        self.assertEqual(session.command(now=100), 0.0)
        values = [session.command(now=100 + index * 0.01) for index in range(2000)]
        self.assertLessEqual(max(abs(value) for value in values), 12.0)
        self.assertEqual(session.command(now=121), 0.0)

    def test_prbs_is_repeatable_and_safe_limits_abort(self):
        first = FrequencyIdentificationSession()
        second = FrequencyIdentificationSession()
        config = {"signal_type": "prbs", "duration": 10, "amplitude": 8, "prbs_rate": 2}
        first.start(config, initial_angle=0, now=0)
        second.start(config, initial_angle=0, now=0)
        self.assertEqual(first.command(now=2.2), second.command(now=2.2))
        pwm = types.SimpleNamespace(biceps=1, triceps=-1, deltoid=0)
        result = first.add_sample({"measured_angle": 130, "measured_angle_raw": 130}, 1, 1, pwm, now=1)
        self.assertEqual(result, "aborted")
        self.assertEqual(first.state, "aborted")

    def test_numpy_frequency_analysis_and_arx(self):
        session = FrequencyIdentificationSession()
        with tempfile.TemporaryDirectory() as directory:
            session.start({"signal_type": "chirp", "duration": 20, "amplitude": 10,
                           "f_start": 0.1, "f_end": 3.0}, initial_angle=0,
                          output_root=directory, now=0)
            dt = 0.01
            y = 0.0
            pwm = types.SimpleNamespace(biceps=0, triceps=0, deltoid=0)
            for index in range(2001):
                now = index * dt
                u = session.command(now=now)
                y += dt * (-3.0 * y + 1.2 * u)
                frame = {"time": now, "measured_angle": y, "measured_angle_raw": y}
                session.add_sample(frame, u, u, pwm, now=now)
            self.assertEqual(session.state, "completed")
            result = session.analyze()
            self.assertGreater(result["summary"]["trusted_frequency_bins"], 2)
            self.assertGreater(result["arx_model"]["validation_fit_percent"], 70.0)
            self.assertTrue(session.csv_path)
            self.assertTrue(session.json_path)
            self.assertTrue(session.png_path)


if __name__ == "__main__":
    unittest.main()
