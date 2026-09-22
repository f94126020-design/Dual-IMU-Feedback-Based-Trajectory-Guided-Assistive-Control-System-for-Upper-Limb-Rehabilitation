import unittest

from imu_control import IMURehabSystem
from muscle_allocator import MuscleAllocator, MusclePWM


class ProfileBControlTests(unittest.TestCase):
    def test_elbow_pair_is_always_opposite_with_ratio_08(self):
        allocator = MuscleAllocator(
            pwm_limit=255,
            antagonist_release_gain=0.8,
            enable_conditioning=False,
        )
        for command in (-100, -20, 0, 20, 100):
            pwm = allocator.allocate("elbow", command, control_profile="three_muscle")
            self.assertLessEqual(pwm.biceps * pwm.triceps, 0)
            if command > 0:
                self.assertEqual(pwm, MusclePWM(command, -round(0.8 * command), 0))
            elif command < 0:
                self.assertEqual(pwm, MusclePWM(-round(0.8 * abs(command)), abs(command), 0))

    def test_slew_limit_and_reverse_deadtime(self):
        allocator = MuscleAllocator(
            antagonist_release_gain=0.8,
            command_filter_tau=0.18,
            wind_slew_rate=120.0,
            release_slew_rate=160.0,
            reverse_deadtime=0.15,
        )
        previous = 0
        now = 0.0
        for _ in range(150):
            pwm = allocator.allocate("elbow", 100, now=now)
            self.assertLessEqual(abs(pwm.biceps - previous), 2)
            self.assertLessEqual(pwm.biceps * pwm.triceps, 0)
            previous = pwm.biceps
            now += 0.01

        outputs = []
        for _ in range(300):
            pwm = allocator.allocate("elbow", -100, now=now)
            outputs.append(pwm)
            self.assertLessEqual(pwm.biceps * pwm.triceps, 0)
            now += 0.01

        first_reverse = next(i for i, pwm in enumerate(outputs) if pwm.triceps > 0)
        zero_run = 0
        for pwm in outputs[:first_reverse]:
            if pwm.biceps == 0 and pwm.triceps == 0:
                zero_run += 1
        self.assertGreaterEqual(zero_run, 12)

    def test_disabled_and_emergency_are_immediate_zero(self):
        allocator = MuscleAllocator()
        allocator.allocate("elbow", 100, now=0.0)
        self.assertEqual(
            allocator.allocate("elbow", 100, motor_enabled=False, now=0.01),
            MusclePWM(),
        )
        allocator.allocate("elbow", 100, now=0.02)
        self.assertEqual(
            allocator.allocate("elbow", 100, emergency_stop=True, now=0.03),
            MusclePWM(),
        )

    def test_shoulder_release_is_velocity_scaled_and_fades_near_down(self):
        system = IMURehabSystem()
        high = system.update_shoulder_release(60.0, -20.0)
        near_down = system.update_shoulder_release(10.0, -20.0)
        stopped = system.update_shoulder_release(10.0, -2.0)
        capped = system.update_shoulder_release(60.0, -200.0)
        self.assertAlmostEqual(high, 22.0)
        self.assertAlmostEqual(near_down, 11.0)
        self.assertEqual(stopped, 0.0)
        self.assertEqual(capped, 60.0)

    def test_pid_hysteresis_d_filter_and_anti_windup(self):
        system = IMURehabSystem(
            controller_mode="pi",
            kp=1.0,
            ki=1.0,
            output_limit=10.0,
            deadband_on=4.0,
            deadband_off=2.0,
        )
        for _ in range(100):
            _, output = system.compute_pid_output(100.0, 0.0, 0.01)
        self.assertEqual(output, 10.0)
        self.assertTrue(system.anti_windup_active)
        self.assertAlmostEqual(system.integral_error, 0.0)

        system.compute_pid_output(3.0, 0.0, 0.01)
        self.assertTrue(system.control_active)
        system.compute_pid_output(1.0, 0.0, 0.01)
        self.assertFalse(system.control_active)

        derivative_system = IMURehabSystem(controller_mode="pid", kp=0.0, ki=0.0, kd=1.0)
        derivative_system.compute_pid_output(0.0, 0.0, 0.01)
        derivative_system.compute_pid_output(0.0, 10.0, 0.01)
        self.assertGreater(derivative_system.derivative_error, -1000.0)
        self.assertLess(derivative_system.derivative_error, 0.0)

    def test_fixed_target_mode(self):
        system = IMURehabSystem(target_mode="fixed", fixed_target_angle=57.5)
        self.assertEqual(system.get_target_angle(0.0), 57.5)
        self.assertEqual(system.get_target_angle(123.0), 57.5)


if __name__ == "__main__":
    unittest.main()
