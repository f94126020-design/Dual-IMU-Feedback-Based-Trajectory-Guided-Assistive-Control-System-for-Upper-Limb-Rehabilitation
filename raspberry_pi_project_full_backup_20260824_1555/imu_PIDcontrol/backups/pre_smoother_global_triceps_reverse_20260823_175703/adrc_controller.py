import math


class LinearADRC:
    """Second-order linear ADRC with a third-order extended state observer."""

    def __init__(self, controller_bandwidth=2.0, observer_bandwidth=8.0,
                 input_gain=1.0, output_limit=50.0):
        self.configure(controller_bandwidth, observer_bandwidth, input_gain, output_limit)
        self.reset()

    def configure(self, controller_bandwidth=None, observer_bandwidth=None,
                  input_gain=None, output_limit=None):
        if controller_bandwidth is not None:
            self.wc = min(max(float(controller_bandwidth), 0.1), 20.0)
        if observer_bandwidth is not None:
            self.wo = min(max(float(observer_bandwidth), 0.3), 60.0)
        if input_gain is not None:
            value = float(input_gain)
            if not math.isfinite(value) or abs(value) < 0.01:
                raise ValueError("ADRC b0 的絕對值必須至少為 0.01。")
            self.b0 = value
        if output_limit is not None:
            self.output_limit = min(max(abs(float(output_limit)), 0.0), 255.0)
        self.kp = self.wc * self.wc
        self.kd = 2.0 * self.wc
        self.beta1 = 3.0 * self.wo
        self.beta2 = 3.0 * self.wo * self.wo
        self.beta3 = self.wo * self.wo * self.wo

    def reset(self, measured_angle=0.0):
        self.z1 = float(measured_angle)
        self.z2 = 0.0
        self.z3 = 0.0
        self.last_u = 0.0
        self.initialized = False
        self.saturated = False

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))

    def update(self, target_angle, target_velocity, measured_angle, dt):
        dt = self._clamp(float(dt), 0.001, 0.05)
        measured = float(measured_angle)
        if not self.initialized:
            self.z1 = measured
            self.z2 = 0.0
            self.z3 = 0.0
            self.last_u = 0.0
            self.initialized = True

        # Substep the ESO when Linux scheduling produces a long sample. This
        # keeps explicit integration stable at higher observer bandwidths.
        substeps = max(1, int(math.ceil(dt * self.wo / 0.2)))
        sub_dt = dt / substeps
        observer_error = 0.0
        for _ in range(substeps):
            observer_error = self.z1 - measured
            z1_dot = self.z2 - self.beta1 * observer_error
            z2_dot = self.z3 - self.beta2 * observer_error + self.b0 * self.last_u
            z3_dot = -self.beta3 * observer_error
            self.z1 += sub_dt * z1_dot
            self.z2 += sub_dt * z2_dot
            self.z3 += sub_dt * z3_dot

        position_error = float(target_angle) - self.z1
        velocity_error = float(target_velocity) - self.z2
        virtual_control = self.kp * position_error + self.kd * velocity_error
        raw_output = (virtual_control - self.z3) / self.b0
        output = self._clamp(raw_output, -self.output_limit, self.output_limit)
        self.saturated = abs(raw_output - output) > 1e-9
        self.last_u = output
        return {
            "output": output,
            "raw_output": raw_output,
            "estimated_angle": self.z1,
            "estimated_velocity": self.z2,
            "estimated_disturbance": self.z3,
            "observer_error": observer_error,
            "position_error": position_error,
            "velocity_error": velocity_error,
            "saturated": self.saturated,
        }
