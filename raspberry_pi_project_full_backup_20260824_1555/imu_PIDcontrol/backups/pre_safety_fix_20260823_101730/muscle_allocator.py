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
        motor_min_pwm=None,
        antagonist_release_gain=1.0,
        triceps_release_ratio=0.5,
        biceps_release_ratio=0.8,
        shoulder_release_pwm=60,
        command_filter_tau=0.02,
        wind_slew_rate=1200.0,
        release_slew_rate=2000.0,
        reverse_deadtime=0.02,
        cable_return_gain=1.0,
        cable_effort_limit=5000.0,
        cable_return_threshold=1.0,
        cable_return_horizon=2.0,
        cable_return_max_pwm=60.0,
        enable_conditioning=True,
        motor_sign=(1, 1, 1),
        biceps_limits=None,
        triceps_limits=None,
        deltoid_limits=None,
    ):
        self.pwm_limit = int(pwm_limit)
        self.active_min_pwm = int(active_min_pwm)
        if motor_min_pwm is None:
            self.motor_min_pwm = [self.active_min_pwm] * 3
        else:
            values = list(motor_min_pwm)
            if len(values) != 3:
                raise ValueError("motor_min_pwm must contain biceps, triceps, deltoid")
            self.motor_min_pwm = [max(0, int(round(value))) for value in values]
        self.antagonist_release_gain = float(antagonist_release_gain)
        self.triceps_release_ratio = self.clamp(float(triceps_release_ratio), 0.05, 1.0)
        self.biceps_release_ratio = self.clamp(float(biceps_release_ratio), 0.05, 1.0)
        self.shoulder_release_pwm = int(shoulder_release_pwm)
        self.command_filter_tau = max(float(command_filter_tau), 0.0)
        self.wind_slew_rate = max(float(wind_slew_rate), 0.0)
        self.release_slew_rate = max(float(release_slew_rate), 0.0)
        self.reverse_deadtime = max(float(reverse_deadtime), 0.0)
        self.cable_return_gain = max(float(cable_return_gain), self.antagonist_release_gain)
        self.cable_effort_limit = max(float(cable_effort_limit), 0.0)
        self.cable_return_threshold = max(float(cable_return_threshold), 0.0)
        self.cable_return_horizon = max(float(cable_return_horizon), 0.1)
        self.cable_return_max_pwm = max(float(cable_return_max_pwm), 0.0)
        self.enable_conditioning = bool(enable_conditioning)
        self.motor_sign = list(motor_sign)
        self.biceps_limits = biceps_limits or EncoderLimits()
        self.triceps_limits = triceps_limits or EncoderLimits()
        self.deltoid_limits = deltoid_limits or EncoderLimits()
        self.reset_conditioner()
        self.reset_cable_tracker()

    def reset_conditioner(self):
        self.conditioner_states = {}
        # Legacy mirrors retain the most recently processed joint for status
        # and backwards compatibility. Control state itself is per joint.
        self.conditioned_command = 0.0
        self.conditioner_last_time = None
        self.conditioner_joint = None
        self.pending_reverse_target = None
        self.reverse_hold_until = None

    def reset_cable_tracker(self):
        """Set the current cable lengths as the zero-effort reference."""
        self.virtual_cable_effort = {
            "biceps": 0.0,
            "triceps": 0.0,
            "deltoid": 0.0,
        }
        self.cable_tracker_last_time = None

    def cable_effort_status(self):
        """Estimated wound cable expressed as integrated logical PWM seconds."""
        return dict(self.virtual_cable_effort)

    def observe_output(self, pwm, now=None):
        """
        Integrate the PWM that was actually sent.

        This is deliberately an estimate, not a position measurement.  Positive
        logical PWM adds wound-cable effort and negative PWM repays it.  Keeping
        it separate from reset_conditioner() means an ordinary STOP does not
        forget cable already wound during the current calibrated session.
        """
        timestamp = time.monotonic() if now is None else float(now)
        if self.cable_tracker_last_time is None:
            self.cable_tracker_last_time = timestamp
            return self.cable_effort_status()
        dt = max(0.0, min(timestamp - self.cable_tracker_last_time, 0.1))
        self.cable_tracker_last_time = timestamp
        for index, name in enumerate(("biceps", "triceps", "deltoid")):
            physical = float(getattr(pwm, name, 0.0))
            sign = self.motor_sign[index] if self.motor_sign[index] else 1
            logical = physical / sign
            effort = self.virtual_cable_effort[name]
            if logical > 0.0:
                effort += logical * dt
            elif logical < 0.0:
                # The measured payout ratios describe unequal cable travel per
                # PWM. Convert release PWM back to equivalent wound-cable PWM
                # so (200,-100) and (-160,200) each repay equal cable travel.
                release_ratio = (
                    self.biceps_release_ratio
                    if index == 0
                    else self.triceps_release_ratio
                    if index == 1
                    else 1.0
                )
                effort -= abs(logical) / release_ratio * dt
            self.virtual_cable_effort[name] = self.clamp(
                effort, 0.0, self.cable_effort_limit
            )
        return self.cable_effort_status()

    def apply_cable_return_compensation(self, pwm, active_magnitude):
        """Repay prior winding during the corresponding reverse-motion phase."""
        active = abs(float(active_magnitude))
        if active <= 0.0:
            return pwm
        biceps = pwm.biceps
        triceps = pwm.triceps
        deltoid = pwm.deltoid
        minimum_return = self.clamp_pwm(active * self.cable_return_gain)
        if (
            biceps < 0
            and self.virtual_cable_effort["biceps"] > self.cable_return_threshold
        ):
            biceps = -max(abs(biceps), minimum_return)
        if (
            triceps < 0
            and self.virtual_cable_effort["triceps"] > self.cable_return_threshold
        ):
            triceps = -max(abs(triceps), minimum_return)
        if (
            deltoid < 0
            and self.virtual_cable_effort["deltoid"] > self.cable_return_threshold
        ):
            estimated_return = self.clamp_pwm(
                min(
                    self.virtual_cable_effort["deltoid"] / self.cable_return_horizon,
                    self.cable_return_max_pwm,
                )
            )
            deltoid = -max(abs(deltoid), estimated_return)
        return MusclePWM(biceps=biceps, triceps=triceps, deltoid=deltoid)

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
        joint = str(target_joint).lower()
        if not self.enable_conditioning:
            self.conditioned_command = target
            self.conditioner_joint = joint
            return target

        timestamp = time.monotonic() if now is None else float(now)
        state = self.conditioner_states.setdefault(joint, {
            "command": 0.0,
            "last_time": None,
            "pending_reverse_target": None,
            "reverse_hold_until": None,
        })
        if state["last_time"] is None:
            dt = 0.01
        else:
            dt = max(0.001, min(timestamp - state["last_time"], 0.1))
        state["last_time"] = timestamp
        current = state["command"]

        if state["pending_reverse_target"] is not None:
            if target == 0.0:
                state["pending_reverse_target"] = None
                state["reverse_hold_until"] = None
            elif self.sign(target) == self.sign(current) and self.sign(current) != 0:
                # The requested reversal was cancelled before reaching zero.
                state["pending_reverse_target"] = None
                state["reverse_hold_until"] = None
            else:
                state["pending_reverse_target"] = target
        elif current * target < 0.0:
            state["pending_reverse_target"] = target
            state["reverse_hold_until"] = None

        effective_target = target
        if state["pending_reverse_target"] is not None:
            effective_target = 0.0
            if abs(current) <= 0.5:
                current = 0.0
                state["command"] = 0.0
                if state["reverse_hold_until"] is None:
                    state["reverse_hold_until"] = timestamp + self.reverse_deadtime
                elif timestamp >= state["reverse_hold_until"]:
                    effective_target = state["pending_reverse_target"]
                    state["pending_reverse_target"] = None
                    state["reverse_hold_until"] = None

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
        state["command"] = float(self.clamp(next_command, -self.pwm_limit, self.pwm_limit))
        self.conditioned_command = state["command"]
        self.conditioner_last_time = state["last_time"]
        self.conditioner_joint = joint
        self.pending_reverse_target = state["pending_reverse_target"]
        self.reverse_hold_until = state["reverse_hold_until"]
        return state["command"]

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def clamp_pwm(self, value):
        return int(self.clamp(int(round(value)), -self.pwm_limit, self.pwm_limit))

    def apply_min_pwm(self, pwm, motor_index=None):
        pwm = self.clamp_pwm(pwm)
        minimum = (
            self.active_min_pwm
            if motor_index is None
            else self.motor_min_pwm[int(motor_index)]
        )
        if pwm == 0 or minimum <= 0:
            return pwm
        if abs(pwm) < minimum:
            return minimum if pwm > 0 else -minimum
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
        enforce_minimum=True,
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

        def shape_output(value, motor_index):
            if enforce_minimum:
                return self.apply_min_pwm(value, motor_index)
            return self.clamp_pwm(value)

        if target_joint == "elbow":
            if u > 0:
                # elbow flexion assistance: biceps winds, triceps releases
                winding = abs(shape_output(u, 0))
                pwm.biceps = winding
                pwm.triceps = -self.clamp_pwm(
                    winding * self.triceps_release_ratio
                )
            elif u < 0:
                # elbow extension assistance: triceps winds, biceps releases
                winding = abs(shape_output(abs(u), 1))
                pwm.biceps = -self.clamp_pwm(
                    winding * self.biceps_release_ratio
                )
                pwm.triceps = winding

        elif target_joint == "shoulder":
            if u > 0:
                pwm.deltoid = shape_output(u, 2)
            elif u < 0:
                pwm.deltoid = (
                    shape_output(u, 2)
                    if requested_command < 0
                    else self.clamp_pwm(u)
                )
            if enforce_minimum:
                pwm = self.apply_cable_return_compensation(pwm, abs(u))

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
