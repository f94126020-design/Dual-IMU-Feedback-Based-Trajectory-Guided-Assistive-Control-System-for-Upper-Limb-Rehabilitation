import math

import numpy as np


class IterativeLearningController:
    """Phase-domain P-type ILC for repeated periodic rehabilitation motion."""

    def __init__(self, bins=200, learning_gain=0.08, forgetting_factor=0.98,
                 q_filter_window=9, update_limit=3.0, learned_limit=20.0,
                 minimum_coverage=0.85):
        self.configure(bins, learning_gain, forgetting_factor, q_filter_window,
                       update_limit, learned_limit, minimum_coverage)
        self.clear_learning()

    def configure(self, bins=None, learning_gain=None, forgetting_factor=None,
                  q_filter_window=None, update_limit=None, learned_limit=None,
                  minimum_coverage=None):
        if bins is not None:
            self.bins = min(max(int(bins), 50), 500)
        if learning_gain is not None:
            self.learning_gain = min(max(float(learning_gain), 0.0), 2.0)
        if forgetting_factor is not None:
            self.forgetting_factor = min(max(float(forgetting_factor), 0.0), 1.0)
        if q_filter_window is not None:
            window = min(max(int(q_filter_window), 1), 51)
            self.q_filter_window = window if window % 2 == 1 else window + 1
        if update_limit is not None:
            self.update_limit = min(max(abs(float(update_limit)), 0.0), 30.0)
        if learned_limit is not None:
            self.learned_limit = min(max(abs(float(learned_limit)), 0.0), 100.0)
        if minimum_coverage is not None:
            self.minimum_coverage = min(max(float(minimum_coverage), 0.5), 1.0)

    def clear_learning(self):
        self.learned = np.zeros(self.bins, dtype=float)
        self.completed_cycles = 0
        self.valid_cycles = 0
        self.invalid_cycles = 0
        self.last_rmse = None
        self.previous_rmse = None
        self.improvement_percent = None
        self.last_cycle_valid = None
        self.last_cycle_reason = ""
        self.frozen = False
        self.begin_run()

    def begin_run(self):
        self.current_cycle = None
        self.error_sum = np.zeros(self.bins, dtype=float)
        self.error_count = np.zeros(self.bins, dtype=int)
        self.cycle_invalid = False
        self.cycle_invalid_reason = ""
        self.current_output = 0.0

    def set_frozen(self, frozen):
        self.frozen = bool(frozen)

    def invalidate_cycle(self, reason):
        self.cycle_invalid = True
        if reason and not self.cycle_invalid_reason:
            self.cycle_invalid_reason = str(reason)

    def abort_current_cycle(self, reason="週期中斷。"):
        if self.current_cycle is not None and int(np.sum(self.error_count)) > 0:
            self.completed_cycles += 1
            self.invalid_cycles += 1
            self.last_cycle_valid = False
            self.last_cycle_reason = str(reason)
        self.begin_run()

    def _smooth_circular(self, values):
        if self.q_filter_window <= 1:
            return values.copy()
        half = self.q_filter_window // 2
        padded = np.concatenate((values[-half:], values, values[:half]))
        kernel = np.ones(self.q_filter_window, dtype=float) / self.q_filter_window
        return np.convolve(padded, kernel, mode="valid")

    def _finalize_cycle(self):
        observed = self.error_count > 0
        coverage = float(np.mean(observed))
        self.completed_cycles += 1
        if self.cycle_invalid or coverage < self.minimum_coverage:
            self.invalid_cycles += 1
            self.last_cycle_valid = False
            self.last_cycle_reason = self.cycle_invalid_reason or f"相位覆蓋率只有 {coverage:.1%}。"
            return

        errors = np.zeros(self.bins, dtype=float)
        errors[observed] = self.error_sum[observed] / self.error_count[observed]
        if not np.all(observed):
            indices = np.flatnonzero(observed)
            errors = np.interp(np.arange(self.bins), indices, errors[indices], period=self.bins)
        rmse = float(math.sqrt(np.mean(errors * errors)))
        self.previous_rmse = self.last_rmse
        self.last_rmse = rmse
        if self.previous_rmse is not None and self.previous_rmse > 1e-9:
            self.improvement_percent = 100.0 * (self.previous_rmse - rmse) / self.previous_rmse

        if not self.frozen:
            delta = np.clip(self.learning_gain * errors, -self.update_limit, self.update_limit)
            candidate = self.forgetting_factor * self.learned + delta
            self.learned = np.clip(
                self._smooth_circular(candidate),
                -self.learned_limit,
                self.learned_limit,
            )
        self.valid_cycles += 1
        self.last_cycle_valid = True
        self.last_cycle_reason = "學習已凍結，僅評估誤差。" if self.frozen else "已更新 learned feedforward。"

    def output_at_phase(self, phase):
        phase = float(phase) % 1.0
        position = phase * self.bins
        low = int(math.floor(position)) % self.bins
        high = (low + 1) % self.bins
        fraction = position - math.floor(position)
        return float((1.0 - fraction) * self.learned[low] + fraction * self.learned[high])

    def update(self, trajectory_time, period, error, valid=True, invalid_reason=""):
        period = max(float(period), 0.1)
        trajectory_time = max(float(trajectory_time), 0.0)
        cycle = int(math.floor(trajectory_time / period))
        phase = (trajectory_time % period) / period
        if self.current_cycle is None:
            self.current_cycle = cycle
        elif cycle != self.current_cycle:
            self._finalize_cycle()
            self.current_cycle = cycle
            self.error_sum.fill(0.0)
            self.error_count.fill(0)
            self.cycle_invalid = False
            self.cycle_invalid_reason = ""

        if not valid:
            self.invalidate_cycle(invalid_reason or "控制周期無效。")
        index = min(int(phase * self.bins), self.bins - 1)
        if math.isfinite(float(error)):
            self.error_sum[index] += float(error)
            self.error_count[index] += 1
        else:
            self.invalidate_cycle("誤差不是有限數值。")
        self.current_output = self.output_at_phase(phase)
        return self.current_output

    def status(self, include_curve=False):
        coverage = float(np.mean(self.error_count > 0)) if self.current_cycle is not None else 0.0
        result = {
            "enabled": True,
            "frozen": self.frozen,
            "current_cycle": self.current_cycle,
            "completed_cycles": self.completed_cycles,
            "valid_cycles": self.valid_cycles,
            "invalid_cycles": self.invalid_cycles,
            "current_cycle_coverage": coverage,
            "last_rmse": self.last_rmse,
            "previous_rmse": self.previous_rmse,
            "improvement_percent": self.improvement_percent,
            "last_cycle_valid": self.last_cycle_valid,
            "last_cycle_reason": self.last_cycle_reason,
            "current_output": self.current_output,
            "learned_peak_pwm": float(np.max(np.abs(self.learned))) if len(self.learned) else 0.0,
            "learning_gain": self.learning_gain,
            "forgetting_factor": self.forgetting_factor,
            "q_filter_window": self.q_filter_window,
            "update_limit": self.update_limit,
            "learned_limit": self.learned_limit,
            "bins": self.bins,
        }
        if include_curve:
            result["phase"] = (np.arange(self.bins) / self.bins).astype(float).tolist()
            result["learned_curve"] = self.learned.astype(float).tolist()
        return result
