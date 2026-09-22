import unittest

from imu_control import IMURehabSystem
from muscle_allocator import MuscleAllocator, MusclePWM


class SingleMotorDryRunTests(unittest.TestCase):
    def setUp(self):
        self.allocator = MuscleAllocator(pwm_limit=30, motor_sign=(1, 1, 1))

    def allocate(self, command, enabled=True, emergency=False, joint="elbow"):
        return self.allocator.allocate(
            target_joint=joint,
            motor_cmd=command,
            control_profile="single_biceps",
            motor_enabled=enabled,
            emergency_stop=emergency,
        )

    def test_only_motor_1_is_allocated(self):
        self.assertEqual(self.allocate(18), MusclePWM(18, 0, 0))
        self.assertEqual(self.allocate(-12), MusclePWM(-12, 0, 0))

    def test_pwm_is_hard_clamped_to_30(self):
        self.assertEqual(self.allocate(999), MusclePWM(30, 0, 0))
        self.assertEqual(self.allocate(-999), MusclePWM(-30, 0, 0))

    def test_disabled_emergency_and_other_joint_are_zero(self):
        self.assertEqual(self.allocate(20, enabled=False), MusclePWM())
        self.assertEqual(self.allocate(20, emergency=True), MusclePWM())
        self.assertEqual(self.allocate(20, joint="shoulder"), MusclePWM())

    def test_fixed_target_mode(self):
        system = IMURehabSystem(target_mode="fixed", fixed_target_angle=57.5)
        self.assertEqual(system.get_target_angle(0.0), 57.5)
        self.assertEqual(system.get_target_angle(123.0), 57.5)


if __name__ == "__main__":
    unittest.main()
