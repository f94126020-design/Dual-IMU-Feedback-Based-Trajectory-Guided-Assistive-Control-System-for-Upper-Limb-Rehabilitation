import unittest

from rom_calibration import ROMCalibrationSession


DT = 0.05


def cycle_value(elapsed, high, low=0.0):
    phase = elapsed % ROMCalibrationSession.CYCLE_DURATION
    out_end = ROMCalibrationSession.OUT_DURATION
    hold_end = out_end + ROMCalibrationSession.HOLD_DURATION
    return_end = hold_end + ROMCalibrationSession.RETURN_DURATION
    if phase < out_end:
        return low + (high - low) * phase / ROMCalibrationSession.OUT_DURATION
    if phase < hold_end:
        return high
    if phase < return_end:
        return high + (low - high) * (phase - hold_end) / ROMCalibrationSession.RETURN_DURATION
    return low


def feed_stage(session, stage, start):
    session.start_stage(stage, now=start)
    duration = session.STAGE_DURATION[stage]
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
            value = cycle_value(elapsed, 100.0, 0.0)
            frame["elbow_roll_signed"] = value
            frame["elbow_pitch_signed"] = 0.1 * value
        elif stage == "elbow_extension":
            value = cycle_value(elapsed, -3.0, 90.0)
            frame["elbow_roll_signed"] = value
            frame["elbow_pitch_signed"] = 0.1 * value
        else:
            maximum = 120.0 if stage == "shoulder_front" else 100.0
            value = cycle_value(elapsed, maximum, 0.0)
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
            now += session.STAGE_DURATION[stage] + 1.0
        return session

    def test_four_task_rom_and_mapping(self):
        session = self.make_complete()
        status = session.status()
        self.assertTrue(status["ready_to_apply"])
        self.assertEqual(status["mapping"]["elbow_axis"], "roll")
        self.assertEqual(status["mapping"]["elbow_sign"], 1)
        self.assertAlmostEqual(status["rom"]["elbow_flexion_deg"], 100.0, delta=1.0)
        self.assertAlmostEqual(status["rom"]["elbow_extension_deg"], -3.0, delta=1.0)
        self.assertAlmostEqual(status["rom"]["shoulder_front_max_deg"], 120.0, delta=1.0)
        self.assertAlmostEqual(status["rom"]["shoulder_side_max_deg"], 100.0, delta=1.0)
        self.assertEqual(status["rom"]["training_fraction"], 0.9)

    def test_apply_returns_task_specific_targets(self):
        result = self.make_complete().apply()
        self.assertGreater(result["rom"]["elbow_flexion_target_deg"], 80.0)
        self.assertLess(result["rom"]["elbow_extension_target_deg"], 15.0)
        self.assertAlmostEqual(result["rom"]["shoulder_front_target_deg"], 108.0, delta=1.0)
        self.assertAlmostEqual(result["rom"]["shoulder_side_target_deg"], 90.0, delta=1.0)

    def test_sequence_starts_with_flexion_then_90_degree_extension(self):
        session = ROMCalibrationSession()
        status = session.start()
        self.assertEqual(status["expected_stage"], "elbow_flexion")
        feed_stage(session, "elbow_flexion", 0.0)
        self.assertEqual(session.status()["expected_stage"], "elbow_extension")
        self.assertIn("90°", session.STAGE_LABELS["elbow_extension"])


if __name__ == "__main__":
    unittest.main()
