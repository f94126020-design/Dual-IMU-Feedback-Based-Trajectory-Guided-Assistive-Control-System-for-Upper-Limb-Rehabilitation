import unittest

from imu_control import IMURehabSystem
from muscle_allocator import MuscleAllocator, MusclePWM


class ProfileBControlTests(unittest.TestCase):
    def test_patient_rom_blocks_only_outward_motion_and_fades_near_limit(self):
        system = IMURehabSystem()
        system.patient_rom = {
            "elbow_extension_target_deg": 0.0,
            "elbow_flexion_target_deg": 90.0,
            "shoulder_front_target_deg": 80.0,
            "shoulder_side_target_deg": 70.0,
        }
        near_high, status = system.limit_command_to_patient_rom("elbow", 87.0, 200.0)
        self.assertEqual(near_high, 120.0)
        self.assertTrue(status["limited"])
        blocked, _ = system.limit_command_to_patient_rom("elbow", 91.0, 200.0)
        self.assertEqual(blocked, 0.0)
        reverse, _ = system.limit_command_to_patient_rom("elbow", 91.0, -200.0)
        self.assertEqual(reverse, -200.0)
        shoulder, _ = system.limit_command_to_patient_rom(
            "shoulder", 68.0, 200.0, "側平舉"
        )
        self.assertEqual(shoulder, 80.0)

    def test_patient_rom_endpoint_latch_has_three_degree_hysteresis(self):
        system = IMURehabSystem()
        system.patient_rom = {
            "elbow_extension_target_deg": 0.0,
            "elbow_flexion_target_deg": 90.0,
            "shoulder_front_target_deg": 80.0,
            "shoulder_side_target_deg": 70.0,
        }
        system.limit_command_to_patient_rom("elbow", 91.0, 200.0)
        still_blocked, _ = system.limit_command_to_patient_rom("elbow", 88.0, 200.0)
        self.assertEqual(still_blocked, 0.0)
        released, _ = system.limit_command_to_patient_rom("elbow", 86.0, 200.0)
        self.assertEqual(released, 160.0)

    def test_default_kp_is_five(self):
        self.assertEqual(IMURehabSystem().kp, 5.0)

    def test_default_following_threshold_rejects_small_motion(self):
        system = IMURehabSystem(controller_mode="following_only")
        self.assertEqual(system.following_velocity_threshold, 5.0)
        self.assertEqual(system.compute_following_output(4.9), 0.0)
        self.assertEqual(system.compute_following_output(-4.9), 0.0)
        self.assertGreater(system.compute_following_output(5.1), 0.0)
        self.assertLess(system.compute_following_output(-5.1), 0.0)

    def test_removed_duplicate_modes_map_to_canonical_controllers(self):
        for old_mode in ("assist", "continuous_assist", "continuous", "feedforward_assist"):
            self.assertEqual(IMURehabSystem(controller_mode=old_mode).controller_mode, "pid")
        self.assertEqual(IMURehabSystem(controller_mode="feedforward_adrc").controller_mode, "adrc")

    def test_each_motor_has_independent_minimum_effective_pwm(self):
        allocator = MuscleAllocator(
            motor_min_pwm=(15, 16, 17),
            enable_conditioning=False,
        )
        self.assertEqual(allocator.allocate("elbow", 5), MusclePWM(15, -8, 0))
        self.assertEqual(allocator.allocate("elbow", -5), MusclePWM(-13, 16, 0))
        self.assertEqual(allocator.allocate("shoulder", 5), MusclePWM(0, 0, 17))
        self.assertEqual(allocator.allocate("shoulder", -5), MusclePWM(0, 0, -17))

    def test_elbow_and_shoulder_conditioners_keep_independent_state(self):
        allocator = MuscleAllocator(
            motor_min_pwm=(0, 0, 0),
            active_min_pwm=0,
            command_filter_tau=0.0,
            wind_slew_rate=100.0,
            release_slew_rate=100.0,
            enable_conditioning=True,
        )
        allocator.condition_command("elbow", 100, now=0.00)
        allocator.condition_command("shoulder", -100, now=0.00)
        elbow_next = allocator.condition_command("elbow", 100, now=0.10)
        shoulder_next = allocator.condition_command("shoulder", -100, now=0.10)
        self.assertGreater(elbow_next, 0.0)
        self.assertLess(shoulder_next, 0.0)
        self.assertIn("elbow", allocator.conditioner_states)
        self.assertIn("shoulder", allocator.conditioner_states)

    def test_shoulder_assist_can_start_from_natural_down_position(self):
        for mode in ("pid", "assist", "continuous_assist", "adrc", "ilc_pid"):
            system = IMURehabSystem(controller_mode=mode)
            self.assertTrue(system.shoulder_tracking_enabled_from_down(), mode)

    def test_elbow_pair_uses_measured_asymmetric_release_ratios(self):
        allocator = MuscleAllocator(
            pwm_limit=255,
            triceps_release_ratio=0.5,
            biceps_release_ratio=0.8,
            enable_conditioning=False,
        )
        for command in (-100, -20, 0, 20, 100):
            pwm = allocator.allocate("elbow", command, control_profile="three_muscle")
            self.assertLessEqual(pwm.biceps * pwm.triceps, 0)
            if command > 0:
                self.assertEqual(pwm, MusclePWM(command, -round(command * 0.5), 0))
            elif command < 0:
                self.assertEqual(pwm, MusclePWM(-round(abs(command) * 0.8), abs(command), 0))

    def test_low_power_test_modes_can_bypass_therapeutic_minimum(self):
        allocator = MuscleAllocator(
            motor_min_pwm=(200, 200, 200),
            enable_conditioning=False,
        )
        self.assertEqual(
            allocator.allocate("elbow", 20, enforce_minimum=False),
            MusclePWM(20, -10, 0),
        )
        self.assertEqual(
            allocator.allocate("shoulder", 15, enforce_minimum=False),
            MusclePWM(0, 0, 15),
        )

    def test_therapeutic_minimum_applies_to_winding_motor_before_release_ratio(self):
        allocator = MuscleAllocator(
            motor_min_pwm=(200, 200, 200),
            triceps_release_ratio=0.5,
            biceps_release_ratio=0.8,
            enable_conditioning=False,
        )
        self.assertEqual(allocator.allocate("elbow", 1), MusclePWM(200, -100, 0))
        self.assertEqual(allocator.allocate("elbow", -1), MusclePWM(-160, 200, 0))

    def test_virtual_cable_effort_is_repaid_on_reverse_phase(self):
        allocator = MuscleAllocator(
            antagonist_release_gain=0.8,
            cable_return_gain=1.0,
            enable_conditioning=False,
        )
        for index in range(101):
            now = index * 0.01
            pwm = allocator.allocate("elbow", 50, now=now)
            allocator.observe_output(pwm, now=now)

        self.assertGreater(allocator.cable_effort_status()["biceps"], 49.0)
        reverse = allocator.allocate("elbow", -50, now=1.01)
        self.assertEqual(reverse.biceps, -40)
        self.assertEqual(reverse.triceps, 50)

        for index in range(101, 202):
            now = index * 0.01
            pwm = allocator.allocate("elbow", -50, now=now)
            allocator.observe_output(pwm, now=now)
        self.assertLess(allocator.cable_effort_status()["biceps"], 1.0)

    def test_stop_does_not_forget_wound_cable(self):
        allocator = MuscleAllocator(enable_conditioning=False)
        allocator.observe_output(MusclePWM(50, 0, 0), now=0.0)
        allocator.observe_output(MusclePWM(50, 0, 0), now=1.0)
        before = allocator.cable_effort_status()["biceps"]
        allocator.allocate("elbow", 0, motor_enabled=False, now=1.1)
        self.assertEqual(allocator.cable_effort_status()["biceps"], before)

    def test_shoulder_payout_uses_virtual_winding_estimate(self):
        allocator = MuscleAllocator(enable_conditioning=False)
        for index in range(11):
            allocator.observe_output(MusclePWM(0, 0, 50), now=index * 0.1)
        pwm = allocator.allocate("shoulder", -10, now=1.01)
        self.assertEqual(pwm.deltoid, -25)

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

    def test_sine_starts_at_minimum_with_zero_velocity(self):
        system = IMURehabSystem(
            target_mode="sine",
            trajectory_min_angle=10.0,
            trajectory_max_angle=90.0,
            trajectory_period=8.0,
        )
        self.assertAlmostEqual(system.get_target_angle(0.0), 10.0)
        self.assertAlmostEqual(system.get_target_velocity(0.0), 0.0)
        self.assertGreater(system.get_target_velocity(2.0), 0.0)

    def test_following_output_uses_measured_joint_velocity(self):
        system = IMURehabSystem(
            controller_mode="continuous_assist",
            assist_feedforward_gain=0.35,
            assist_feedforward_min_pwm=8.0,
            assist_feedforward_velocity_threshold=2.0,
            following_velocity_threshold=2.0,
        )
        self.assertEqual(system.compute_following_output(0.0), 0.0)
        self.assertEqual(system.compute_following_output(3.0), 8.0)
        self.assertAlmostEqual(system.compute_following_output(40.0), 14.0)
        self.assertAlmostEqual(system.compute_following_output(-40.0), -14.0)

    def test_elbow_following_velocity_uses_calibrated_angle_direction(self):
        system = IMURehabSystem(derivative_filter_tau=0.08)
        self.assertEqual(system.update_elbow_following_velocity(10.0, 0.02), 0.0)
        self.assertGreater(system.update_elbow_following_velocity(12.0, 0.02), 0.0)
        # A sufficiently large decrease must become extension (negative),
        # independent of the separate relative-gyro direction.
        self.assertLess(system.update_elbow_following_velocity(8.0, 0.02), 0.0)

    def test_elbow_following_velocity_resets_with_runtime_state(self):
        system = IMURehabSystem()
        system.update_elbow_following_velocity(20.0, 0.02)
        system.update_elbow_following_velocity(25.0, 0.02)
        self.assertGreater(system.elbow_angle_velocity, 0.0)
        system.reset_runtime_states()
        self.assertEqual(system.elbow_angle_velocity, 0.0)
        self.assertFalse(system.elbow_velocity_initialized)

    def test_all_therapeutic_controllers_have_imu_following(self):
        system = IMURehabSystem(controller_mode="assist")
        self.assertGreater(system.compute_assist_feedforward(40.0), 0.0)
        for mode in ("p", "pi", "pid", "assist", "adrc", "ilc_pid", "ilc_adrc"):
            system.controller_mode = mode
            self.assertGreater(system.compute_assist_feedforward(40.0), 0.0)

    def test_following_only_uses_imu_motion_without_restarted_trajectory(self):
        system = IMURehabSystem(controller_mode="following_only")
        self.assertEqual(system.controller_mode, "following_only")
        self.assertEqual(system.compute_following_output(0.0), 0.0)
        self.assertEqual(system.compute_following_output(20.0), 200.0)
        self.assertEqual(system.compute_following_output(-20.0), -200.0)
        self.assertFalse(system.uses_restarted_trajectory())

    def test_demo_output_uses_velocity_only_and_is_capped(self):
        system = IMURehabSystem(
            controller_mode="trajectory_demo",
            demo_feedforward_gain=1.0,
            demo_min_pwm=8.0,
            demo_output_limit=15.0,
            output_limit=50.0,
        )
        self.assertEqual(system.compute_demo_feedforward(0.0), 0.0)
        self.assertEqual(system.compute_demo_feedforward(3.0), 8.0)
        self.assertEqual(system.compute_demo_feedforward(100.0), 15.0)
        self.assertEqual(system.compute_demo_feedforward(-100.0), -15.0)
        system.target_mode = "step"
        self.assertEqual(system.compute_demo_feedforward(100.0), 0.0)

    def test_demo_limit_is_independent_from_feedback_compensation_limit(self):
        system = IMURehabSystem(
            controller_mode="trajectory_demo",
            demo_feedforward_gain=2.0,
            demo_min_pwm=8.0,
            demo_output_limit=100.0,
            output_limit=55.0,
        )
        self.assertEqual(system.compute_demo_feedforward(100.0), 100.0)
        self.assertEqual(system.compute_demo_feedforward(-100.0), -100.0)

    def test_demo_clock_has_one_second_dwell_and_restarts(self):
        system = IMURehabSystem(controller_mode="trajectory_demo", demo_start_delay=1.0)
        system.reset_trajectory(now=10.0)
        self.assertEqual(system.get_trajectory_time(10.5), 0.0)
        self.assertAlmostEqual(system.get_trajectory_time(12.5), 1.5)
        system.reset_trajectory(now=20.0)
        self.assertEqual(system.get_trajectory_time(20.2), 0.0)
        system.stop_trajectory()
        self.assertEqual(system.get_trajectory_time(99.0), 0.0)

    def test_adrc_configuration_and_feedforward_modes(self):
        system = IMURehabSystem(
            controller_mode="adrc",
            adrc_controller_bandwidth=2.5,
            adrc_observer_bandwidth=9.0,
            adrc_input_gain=1.2,
        )
        self.assertAlmostEqual(system.adrc.wc, 2.5)
        self.assertAlmostEqual(system.adrc.wo, 9.0)
        self.assertNotEqual(system.compute_assist_feedforward(30.0), 0.0)
        system.controller_mode = "adrc"
        self.assertNotEqual(system.compute_assist_feedforward(30.0), 0.0)

    def test_lag_compensation_only_assists_in_motion_direction(self):
        system = IMURehabSystem(controller_mode="pid", output_limit=55.0)
        behind_up = system.compute_tracking_state(60.0, 50.0, 20.0)
        self.assertTrue(behind_up["lag_active"])
        self.assertEqual(system.gate_lag_compensation(80.0, behind_up), 55.0)
        self.assertEqual(system.gate_lag_compensation(-30.0, behind_up), 0.0)

        ahead_up = system.compute_tracking_state(60.0, 65.0, 20.0)
        self.assertFalse(ahead_up["lag_active"])
        self.assertEqual(system.gate_lag_compensation(50.0, ahead_up), 0.0)

        behind_down = system.compute_tracking_state(40.0, 50.0, -20.0)
        self.assertTrue(behind_down["lag_active"])
        self.assertEqual(system.gate_lag_compensation(-30.0, behind_down), -30.0)
        self.assertEqual(system.gate_lag_compensation(30.0, behind_down), 0.0)

        not_large_enough = system.compute_tracking_state(60.0, 57.0, 20.0)
        self.assertFalse(not_large_enough["lag_active"])
        threshold_reached = system.compute_tracking_state(60.0, 56.0, 20.0)
        self.assertTrue(threshold_reached["lag_active"])

        system.control_active = True
        hysteresis_holds = system.compute_tracking_state(60.0, 57.0, 20.0)
        self.assertTrue(hysteresis_holds["lag_active"])
        hysteresis_releases = system.compute_tracking_state(60.0, 58.0, 20.0)
        self.assertFalse(hysteresis_releases["lag_active"])

    def test_low_base_plus_high_lag_boost_reaches_physical_limit(self):
        system = IMURehabSystem(
            controller_mode="pid",
            assist_feedforward_min_pwm=200.0,
            output_limit=55.0,
        )
        base = system.compute_assist_feedforward(20.0)
        behind = system.compute_tracking_state(60.0, 40.0, 20.0)
        boost = system.gate_lag_compensation(100.0, behind)
        self.assertEqual(base, 200.0)
        self.assertEqual(boost, 55.0)
        self.assertEqual(system.combine_tracking_output(base, boost), 255.0)

        ahead = system.compute_tracking_state(60.0, 65.0, 20.0)
        no_boost = system.gate_lag_compensation(100.0, ahead)
        self.assertEqual(no_boost, 0.0)
        self.assertEqual(system.combine_tracking_output(base, no_boost), 200.0)

    def test_lag_assistance_overrides_opposing_follow_direction(self):
        system = IMURehabSystem(
            controller_mode="pid",
            assist_feedforward_min_pwm=200.0,
            output_limit=55.0,
        )
        self.assertEqual(
            system.combine_following_and_compensation(-200.0, 40.0),
            240.0,
        )
        self.assertEqual(
            system.combine_following_and_compensation(200.0, -40.0),
            -240.0,
        )
        self.assertEqual(
            system.combine_following_and_compensation(-200.0, 0.0),
            -200.0,
        )

    def test_ilc_mode_uses_restarted_sine_trajectory(self):
        system = IMURehabSystem(
            controller_mode="ilc_pid",
            target_mode="sine",
            ilc_learning_gain=0.1,
            ilc_learned_limit=12,
        )
        self.assertTrue(system.is_ilc_mode())
        self.assertTrue(system.uses_restarted_trajectory())
        system.reset_trajectory(now=10.0)
        self.assertEqual(system.get_trajectory_time(10.5), 0.0)
        self.assertAlmostEqual(system.get_trajectory_time(12.0), 1.0)
        self.assertAlmostEqual(system.ilc.learning_gain, 0.1)
        self.assertAlmostEqual(system.ilc.learned_limit, 12)


if __name__ == "__main__":
    unittest.main()
