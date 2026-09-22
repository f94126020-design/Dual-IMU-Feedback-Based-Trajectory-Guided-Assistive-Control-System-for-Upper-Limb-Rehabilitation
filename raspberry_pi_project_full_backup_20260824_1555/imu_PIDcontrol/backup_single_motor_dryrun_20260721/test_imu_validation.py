import math
import unittest

from imu_validation import IMUValidationSession


DT = 0.05


def phase_angle(elapsed, amplitude=60.0):
    phase = elapsed % 10.0
    if phase < 3.0:
        return amplitude * phase / 3.0
    if phase < 5.0:
        return amplitude
    if phase < 8.0:
        return amplitude * (1.0 - (phase - 5.0) / 3.0)
    return 0.0


def feed_static(session, start=0.0, drift=0.0):
    session.start_stage("static", now=start)
    steps = int(30.0 / DT)
    for index in range(steps):
        elapsed = index * DT
        noise = 0.15 * math.sin(index * 0.31)
        offset = drift * elapsed / 30.0
        session.add_sample({
            "elbow_roll_signed": noise + offset,
            "elbow_pitch_signed": 0.8 * noise + offset,
            "shoulder_angle": 0.5 * noise + offset,
            "shoulder_dx": 0.0,
            "shoulder_dy": 0.0,
            "shoulder_dz": 0.0,
        }, now=start + elapsed)
    return session.finish_stage(now=start + 30.0)


def feed_motion(session, stage, start, elbow_sign=1.0, amplitude=60.0):
    session.start_stage(stage, now=start)
    steps = int(50.0 / DT)
    for index in range(steps):
        elapsed = index * DT
        angle = phase_angle(elapsed, amplitude)
        noise = 0.08 * math.sin(index * 0.27)
        frame = {
            "elbow_roll_signed": noise,
            "elbow_pitch_signed": noise,
            "shoulder_angle": noise,
            "shoulder_dx": 0.0,
            "shoulder_dy": 0.0,
            "shoulder_dz": 0.0,
        }
        if stage == "elbow":
            frame["elbow_roll_signed"] = elbow_sign * angle + noise
            frame["elbow_pitch_signed"] = elbow_sign * angle * 0.05 + noise
        else:
            frame["shoulder_angle"] = angle + noise
            scale = angle / max(amplitude, 1.0)
            if stage == "shoulder_front":
                frame["shoulder_dx"] = scale
                frame["shoulder_dy"] = 0.04 * scale
                frame["shoulder_dz"] = 0.02 * scale
            else:
                frame["shoulder_dx"] = 0.02 * scale
                frame["shoulder_dy"] = 0.04 * scale
                frame["shoulder_dz"] = scale
        session.add_sample(frame, now=start + elapsed)
    return session.finish_stage(now=start + 50.0)


class IMUValidationTests(unittest.TestCase):
    def make_complete_session(self, elbow_sign=1.0):
        session = IMUValidationSession()
        session.start()
        feed_static(session, 0.0)
        feed_motion(session, "elbow", 40.0, elbow_sign=elbow_sign)
        feed_motion(session, "shoulder_front", 100.0)
        feed_motion(session, "shoulder_side", 160.0)
        return session

    def test_complete_validation_passes_and_applies(self):
        session = self.make_complete_session()
        status = session.status(now=220.0)
        self.assertTrue(status["can_apply"])
        self.assertTrue(status["passed"])
        self.assertFalse(status["applied"])
        self.assertEqual(status["recommendation"]["elbow_axis"], "roll")
        self.assertEqual(status["recommendation"]["elbow_sign"], 1)
        mapping = session.apply()
        self.assertEqual(mapping["elbow_axis"], "roll")
        self.assertEqual(mapping["elbow_sign"], 1)
        self.assertTrue(session.status()["applied"])

    def test_reversed_mount_recommends_negative_sign(self):
        session = self.make_complete_session(elbow_sign=-1.0)
        status = session.status(now=220.0)
        self.assertTrue(status["can_apply"])
        self.assertEqual(status["recommendation"]["elbow_sign"], -1)

    def test_low_elbow_amplitude_fails(self):
        session = IMUValidationSession()
        session.start()
        feed_static(session, 0.0)
        status = feed_motion(session, "elbow", 40.0, amplitude=10.0)
        self.assertFalse(status["results"]["elbow"]["passed"])
        self.assertTrue(any("幅度不足" in r for r in status["results"]["elbow"]["reasons"]))

    def test_static_drift_fails(self):
        session = IMUValidationSession()
        session.start()
        status = feed_static(session, 0.0, drift=8.0)
        self.assertFalse(status["results"]["static"]["passed"])
        self.assertTrue(status["results"]["static"]["reasons"])

    def test_data_gap_fails(self):
        session = IMUValidationSession()
        session.start()
        session.start_stage("static", now=0.0)
        for second in range(30):
            session.add_sample({
                "elbow_roll_signed": 0.0,
                "elbow_pitch_signed": 0.0,
                "shoulder_angle": 0.0,
            }, now=float(second))
        status = session.finish_stage(now=30.0)
        self.assertFalse(status["results"]["static"]["passed"])
        self.assertTrue(any("中斷" in r for r in status["results"]["static"]["reasons"]))


if __name__ == "__main__":
    unittest.main()
