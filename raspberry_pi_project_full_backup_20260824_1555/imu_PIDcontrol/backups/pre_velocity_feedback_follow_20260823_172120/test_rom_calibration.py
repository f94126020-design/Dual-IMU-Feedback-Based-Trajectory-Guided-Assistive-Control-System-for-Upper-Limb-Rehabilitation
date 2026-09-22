import unittest

from rom_calibration import ROMCalibrationSession


DT = 0.05


def free_motion_value(elapsed, high, low=0.0):
    if elapsed <= 1.0:
        return low
    phase = (elapsed - 1.0) % 2.0
    if phase < 1.0:
        return low + (high - low) * phase
    return high + (low - high) * (phase - 1.0)


def feed_stage(session, stage, start):
    session.start_stage(stage, now=start)
    duration = session.stage_duration_seconds
    for index in range(int(duration / DT)):
        elapsed = index * DT
        frame = {
            "elbow_roll_signed": 0.0,
            "elbow_pitch_signed": 0.0,
            "shoulder_angle": 0.0,
            "shoulder_dx": 0.0,
            "shoulder_dy": 0.0,
            "shoulder_dz": 0.0,
        }
        if stage == "elbow_flexion":
            value = free_motion_value(elapsed, 100.0, 0.0)
            frame["elbow_roll_signed"] = value
            frame["elbow_pitch_signed"] = 0.1 * value
        elif stage == "elbow_extension":
            value = free_motion_value(elapsed, -3.0, 90.0)
            frame["elbow_roll_signed"] = value
            frame["elbow_pitch_signed"] = 0.1 * value
        else:
            maximum = 120.0 if stage == "shoulder_front" else 100.0
            value = free_motion_value(elapsed, maximum, 0.0)
            if stage == "shoulder_front" and index == int(duration / (2 * DT)):
                value = 999.0  # one synthetic IMU spike must not define the ROM
            frame["shoulder_angle"] = value
            scale = value / maximum
            if stage == "shoulder_front":
                frame["shoulder_dx"] = scale
                frame["shoulder_dy"] = 0.05 * scale
            else:
                frame["shoulder_dz"] = scale
                frame["shoulder_dy"] = 0.05 * scale
        session.add_sample(frame, now=start + elapsed)
    return session.finish_stage(now=start + duration)


class ROMCalibrationTests(unittest.TestCase):
    def make_complete(self):
        session = ROMCalibrationSession()
        session.start()
        now = 0.0
        for stage in session.STAGE_ORDER:
            feed_stage(session, stage, now)
            now += session.stage_duration_seconds + 1.0
        return session

    def test_four_task_rom_and_mapping(self):
        session = self.make_complete()
        status = session.status()
        self.assertTrue(status["ready_to_apply"])
        self.assertEqual(status["mapping"]["elbow_axis"], "roll")
        self.assertEqual(status["mapping"]["elbow_sign"], 1)
        self.assertAlmostEqual(status["rom"]["elbow_flexion_deg"], 100.0, delta=6.0)
        self.assertAlmostEqual(status["rom"]["elbow_extension_deg"], -3.0, delta=6.0)
        self.assertAlmostEqual(status["rom"]["shoulder_front_max_deg"], 120.0, delta=6.0)
        self.assertAlmostEqual(status["rom"]["shoulder_side_max_deg"], 100.0, delta=6.0)
        self.assertEqual(status["rom"]["training_fraction"], 0.9)

    def test_apply_returns_task_specific_targets(self):
        result = self.make_complete().apply()
        self.assertGreater(result["rom"]["elbow_flexion_target_deg"], 80.0)
        self.assertLess(result["rom"]["elbow_extension_target_deg"], 15.0)
        self.assertAlmostEqual(result["rom"]["shoulder_front_target_deg"], 103.0, delta=6.0)
        self.assertAlmostEqual(result["rom"]["shoulder_side_target_deg"], 86.0, delta=5.0)

    def test_sequence_starts_with_flexion_then_90_degree_extension(self):
        session = ROMCalibrationSession()
        status = session.start()
        self.assertEqual(status["expected_stage"], "elbow_flexion")
        feed_stage(session, "elbow_flexion", 0.0)
        self.assertEqual(session.status()["expected_stage"], "elbow_extension")
        self.assertIn("90°", session.STAGE_LABELS["elbow_extension"])

    def test_duration_is_configurable_and_clamped(self):
        session = ROMCalibrationSession()
        self.assertEqual(session.start(3)["stage_duration_seconds"], 5.0)
        self.assertEqual(session.start(12)["stage_duration_seconds"], 12.0)
        self.assertEqual(session.start(99)["stage_duration_seconds"], 30.0)

    def test_single_angle_spike_does_not_define_rom(self):
        session = self.make_complete()
        self.assertLess(session.status()["rom"]["shoulder_front_max_deg"], 150.0)


if __name__ == "__main__":
    unittest.main()
