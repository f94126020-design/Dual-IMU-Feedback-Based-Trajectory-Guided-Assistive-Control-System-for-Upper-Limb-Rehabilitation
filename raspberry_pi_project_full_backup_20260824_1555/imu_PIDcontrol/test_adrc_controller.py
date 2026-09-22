import unittest

from adrc_controller import LinearADRC


class ADRCControllerTests(unittest.TestCase):
    def test_output_is_finite_and_saturated(self):
        controller = LinearADRC(controller_bandwidth=3, observer_bandwidth=10,
                                input_gain=1, output_limit=15)
        result = controller.update(100, 0, 0, 0.01)
        self.assertEqual(result["output"], 15)
        self.assertTrue(result["saturated"])

    def test_observer_tracks_constant_measurement(self):
        controller = LinearADRC(controller_bandwidth=1.5, observer_bandwidth=6,
                                input_gain=1, output_limit=30)
        for _ in range(400):
            result = controller.update(20, 0, 20, 0.01)
        self.assertAlmostEqual(result["estimated_angle"], 20, delta=0.2)
        self.assertAlmostEqual(result["estimated_velocity"], 0, delta=0.5)
        self.assertAlmostEqual(result["output"], 0, delta=1.0)

    def test_reset_is_bumpless_at_current_measurement(self):
        controller = LinearADRC()
        controller.update(100, 0, 0, 0.01)
        controller.reset(35)
        result = controller.update(35, 0, 35, 0.01)
        self.assertAlmostEqual(result["output"], 0.0, delta=1e-9)

    def test_invalid_input_gain_is_rejected(self):
        with self.assertRaises(ValueError):
            LinearADRC(input_gain=0)

    def test_high_bandwidth_long_sample_remains_finite(self):
        controller = LinearADRC(controller_bandwidth=10, observer_bandwidth=60,
                                input_gain=1, output_limit=30)
        for _ in range(100):
            result = controller.update(20, 0, 10, 0.05)
            self.assertTrue(abs(result["estimated_angle"]) < 1e6)
            self.assertTrue(abs(result["estimated_disturbance"]) < 1e8)


if __name__ == "__main__":
    unittest.main()
