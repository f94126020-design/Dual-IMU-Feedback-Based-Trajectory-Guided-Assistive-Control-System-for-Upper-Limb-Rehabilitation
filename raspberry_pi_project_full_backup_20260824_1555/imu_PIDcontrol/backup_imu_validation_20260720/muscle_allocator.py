# muscle_allocator.py
# Motor 1: biceps, Motor 2: triceps, Motor 3: deltoid
# convention: +PWM = wind/tension cable, -PWM = release cable

from dataclasses import dataclass


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
        antagonist_release_gain=0.35,
        shoulder_release_pwm=50,
        motor_sign=(1, 1, 1),
        biceps_limits=None,
        triceps_limits=None,
        deltoid_limits=None,
    ):
        self.pwm_limit = int(pwm_limit)
        self.active_min_pwm = int(active_min_pwm)
        self.antagonist_release_gain = float(antagonist_release_gain)
        self.shoulder_release_pwm = int(shoulder_release_pwm)
        self.motor_sign = list(motor_sign)
        self.biceps_limits = biceps_limits or EncoderLimits()
        self.triceps_limits = triceps_limits or EncoderLimits()
        self.deltoid_limits = deltoid_limits or EncoderLimits()

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
        motor_enabled=True,
        emergency_stop=False,
        shoulder_release_command=False,
        shoulder_motor_enable=True,
        feedback=None,
    ):
        if emergency_stop or not motor_enabled:
            return MusclePWM(0, 0, 0)

        u = self.clamp_pwm(motor_cmd)
        pwm = MusclePWM(0, 0, 0)

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
            if shoulder_release_command:
                # user is lowering the arm: deltoid releases cable
                pwm.deltoid = -abs(self.shoulder_release_pwm)
            else:
                if not shoulder_motor_enable:
                    pwm.deltoid = 0
                elif u > 0:
                    # shoulder elevation assistance: deltoid winds
                    pwm.deltoid = self.apply_min_pwm(u)
                elif u < 0:
                    # target asks to lower: deltoid releases
                    pwm.deltoid = -int(abs(u) * self.antagonist_release_gain)

        # safety: biceps and triceps must not both wind at the same time
        if pwm.biceps > 0 and pwm.triceps > 0:
            if pwm.biceps >= pwm.triceps:
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
    allocator = MuscleAllocator(antagonist_release_gain=0.35, shoulder_release_pwm=50)
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