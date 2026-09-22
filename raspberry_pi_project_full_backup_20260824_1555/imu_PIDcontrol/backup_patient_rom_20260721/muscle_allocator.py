# muscle_allocator.py
# Motor 1: biceps, Motor 2: triceps, Motor 3: deltoid
# convention: +PWM = wind/tension cable, -PWM = release cable

from dataclasses import dataclass
import time


@dataclass
class MusclePWM:
    biceps: int = 0
    triceps: int = 0
    deltoid: int = 0


@dataclass
class EncoderLimits:
    min_count: int = -999999
    max_count: int = 999999


class MuscleAllocator:
    def __init__(
        self,
        pwm_limit=255,
        active_min_pwm=0,
        antagonist_release_gain=0.8,
        shoulder_release_pwm=60,
        command_filter_tau=0.18,
        wind_slew_rate=120.0,
        release_slew_rate=160.0,
        reverse_deadtime=0.15,
        enable_conditioning=True,
        motor_sign=(1, 1, 1),
        biceps_limits=None,
        triceps_limits=None,
        deltoid_limits=None,
    ):
        self.pwm_limit = int(pwm_limit)
        self.active_min_pwm = int(active_min_pwm)
        self.antagonist_release_gain = float(antagonist_release_gain)
        self.shoulder_release_pwm = int(shoulder_release_pwm)
        self.command_filter_tau = max(float(command_filter_tau), 0.0)
        self.wind_slew_rate = max(float(wind_slew_rate), 0.0)
        self.release_slew_rate = max(float(release_slew_rate), 0.0)
        self.reverse_deadtime = max(float(reverse_deadtime), 0.0)
        self.enable_conditioning = bool(enable_conditioning)
        self.motor_sign = list(motor_sign)
        self.biceps_limits = biceps_limits or EncoderLimits()
        self.triceps_limits = triceps_limits or EncoderLimits()
        self.deltoid_limits = deltoid_limits or EncoderLimits()
        self.reset_conditioner()

    def reset_conditioner(self):
        self.conditioned_command = 0.0
        self.conditioner_last_time = None
        self.conditioner_joint = None
        self.pending_reverse_target = None
        self.reverse_hold_until = None

    @staticmethod
    def sign(value):
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    def condition_command(self, target_joint, target_command, now=None):
        """Profile B: low-pass, asymmetric slew rate, and zero-cross deadtime."""
        target = float(self.clamp_pwm(target_command))
        if not self.enable_conditioning:
            self.conditioned_command = target
            return target

        timestamp = time.monotonic() if now is None else float(now)
        if self.conditioner_joint != target_joint:
            self.reset_conditioner()
            self.conditioner_joint = target_joint
            self.conditioner_last_time = timestamp

        if self.conditioner_last_time is None:
            dt = 0.01
        else:
            dt = max(0.001, min(timestamp - self.conditioner_last_time, 0.1))
        self.conditioner_last_time = timestamp
        current = self.conditioned_command

        if self.pending_reverse_target is not None:
            if target == 0.0:
                self.pending_reverse_target = None
                self.reverse_hold_until = None
            elif self.sign(target) == self.sign(current) and self.sign(current) != 0:
                # The requested reversal was cancelled before reaching zero.
                self.pending_reverse_target = None
                self.reverse_hold_until = None
            else:
                self.pending_reverse_target = target
        elif current * target < 0.0:
            self.pending_reverse_target = target
            self.reverse_hold_until = None

        effective_target = target
        if self.pending_reverse_target is not None:
            effective_target = 0.0
            if abs(current) <= 0.5:
                current = 0.0
                self.conditioned_command = 0.0
                if self.reverse_hold_until is None:
                    self.reverse_hold_until = timestamp + self.reverse_deadtime
                elif timestamp >= self.reverse_hold_until:
                    effective_target = self.pending_reverse_target
                    self.pending_reverse_target = None
                    self.reverse_hold_until = None

        if self.command_filter_tau <= 0.0:
            filtered_target = effective_target
        else:
            alpha = dt / (self.command_filter_tau + dt)
            filtered_target = current + alpha * (effective_target - current)

        increasing_magnitude = abs(filtered_target) > abs(current)
        if target_joint == "shoulder" and filtered_target < 0.0:
            rate = self.release_slew_rate
        else:
            rate = self.wind_slew_rate if increasing_magnitude else self.release_slew_rate
        max_step = rate * dt
        next_command = current + self.clamp(
            filtered_target - current,
            -max_step,
            max_step,
        )
        if effective_target == 0.0 and abs(next_command) < 0.5:
            next_command = 0.0
        self.conditioned_command = float(self.clamp(next_command, -self.pwm_limit, self.pwm_limit))
        return self.conditioned_command

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def clamp_pwm(self, value):
        return int(self.clamp(int(round(value)), -self.pwm_limit, self.pwm_limit))

    def apply_min_pwm(self, pwm):
        pwm = self.clamp_pwm(pwm)
        if pwm == 0 or self.active_min_pwm <= 0:
            return pwm
        if abs(pwm) < self.active_min_pwm:
            return self.active_min_pwm if pwm > 0 else -self.active_min_pwm
        return pwm

    def apply_motor_sign(self, pwm):
        return MusclePWM(
            biceps=self.clamp_pwm(pwm.biceps * self.motor_sign[0]),
            triceps=self.clamp_pwm(pwm.triceps * self.motor_sign[1]),
            deltoid=self.clamp_pwm(pwm.deltoid * self.motor_sign[2]),
        )

    def apply_encoder_limits(self, pwm, feedback=None):
        if feedback is None:
            return pwm

        biceps = pwm.biceps
        triceps = pwm.triceps
        deltoid = pwm.deltoid

        c1 = getattr(feedback, "count1", 0)
        c2 = getattr(feedback, "count2", 0)
        c3 = getattr(feedback, "count3", 0)

        # count >= max: no more +PWM winding
        # count <= min: no more -PWM releasing
        if c1 >= self.biceps_limits.max_count and biceps > 0:
            biceps = 0
        if c1 <= self.biceps_limits.min_count and biceps < 0:
            biceps = 0

        if c2 >= self.triceps_limits.max_count and triceps > 0:
            triceps = 0
        if c2 <= self.triceps_limits.min_count and triceps < 0:
            triceps = 0

        if c3 >= self.deltoid_limits.max_count and deltoid > 0:
            deltoid = 0
        if c3 <= self.deltoid_limits.min_count and deltoid < 0:
            deltoid = 0

        return MusclePWM(
            biceps=self.clamp_pwm(biceps),
            triceps=self.clamp_pwm(triceps),
            deltoid=self.clamp_pwm(deltoid),
        )

    def allocate(
        self,
        target_joint,
        motor_cmd,
        control_profile="three_muscle",
        motor_enabled=True,
        emergency_stop=False,
        shoulder_release_command=False,
        shoulder_release_pwm=None,
        shoulder_motor_enable=True,
        feedback=None,
        now=None,
    ):
        if emergency_stop or not motor_enabled:
            self.reset_conditioner()
            return MusclePWM(0, 0, 0)

        requested_command = self.clamp_pwm(motor_cmd)
        if target_joint == "shoulder":
            if shoulder_release_command:
                payout = (
                    self.shoulder_release_pwm
                    if shoulder_release_pwm is None
                    else abs(float(shoulder_release_pwm))
                )
                requested_command = -abs(payout)
            elif not shoulder_motor_enable:
                requested_command = 0
            elif requested_command < 0:
                requested_command = -abs(requested_command) * self.antagonist_release_gain

        u = self.condition_command(target_joint, requested_command, now=now)
        pwm = MusclePWM(0, 0, 0)

        if control_profile == "single_biceps":
            # Phase 2 commissioning profile: one elbow joint, Motor 1 only.
            # Positive PWM winds and negative PWM releases the biceps cable.
            # Encoder limits are intentionally not used here until the encoder
            # inputs have been wired and verified; disconnected inputs float.
            if target_joint == "elbow":
                pwm.biceps = self.apply_min_pwm(u)
            pwm = self.apply_motor_sign(pwm)
            return MusclePWM(
                biceps=self.clamp_pwm(pwm.biceps),
                triceps=0,
                deltoid=0,
            )

        if target_joint == "elbow":
            if u > 0:
                # elbow flexion assistance: biceps winds, triceps releases
                pwm.biceps = self.apply_min_pwm(u)
                pwm.triceps = -int(abs(u) * self.antagonist_release_gain)
            elif u < 0:
                # elbow extension assistance: triceps winds, biceps releases
                pwm.biceps = -int(abs(u) * self.antagonist_release_gain)
                pwm.triceps = self.apply_min_pwm(abs(u))

        elif target_joint == "shoulder":
            if u > 0:
                pwm.deltoid = self.apply_min_pwm(u)
            elif u < 0:
                pwm.deltoid = self.clamp_pwm(u)

        # Pair invariant: biceps and triceps may be opposite or zero, never
        # receive the same logical cable direction.
        if pwm.biceps * pwm.triceps > 0:
            if abs(pwm.biceps) >= abs(pwm.triceps):
                pwm.triceps = 0
            else:
                pwm.biceps = 0

        pwm = MusclePWM(
            biceps=self.clamp_pwm(pwm.biceps),
            triceps=self.clamp_pwm(pwm.triceps),
            deltoid=self.clamp_pwm(pwm.deltoid),
        )
        pwm = self.apply_encoder_limits(pwm, feedback)
        pwm = self.apply_motor_sign(pwm)
        return pwm


if __name__ == "__main__":
    allocator = MuscleAllocator(antagonist_release_gain=0.8, shoulder_release_pwm=60)
    tests = [
        ("elbow", 100, False),
        ("elbow", -100, False),
        ("shoulder", 100, False),
        ("shoulder", -100, False),
        ("shoulder", 0, True),
    ]
    for joint, cmd, release in tests:
        pwm = allocator.allocate(
            target_joint=joint,
            motor_cmd=cmd,
            motor_enabled=True,
            emergency_stop=False,
            shoulder_release_command=release,
            shoulder_motor_enable=True,
        )
        print(joint, cmd, release, "=>", pwm)
