"""Guided, motor-safe IMU mounting and direction validation.

This module intentionally does not assess absolute angle accuracy.  It checks
noise, drift, repeatability, motion direction, the dominant elbow axis, and
whether labelled shoulder front/side raises can be separated.
"""

from __future__ import annotations

import math
import statistics
import threading
import time


class IMUValidationSession:
    STAGE_ORDER = ("static", "elbow", "shoulder_front", "shoulder_side")
    STATIC_DURATION = 6.0
    OUT_DURATION = 2.0
    HOLD_DURATION = 1.0
    RETURN_DURATION = 2.0
    NEUTRAL_DURATION = 1.0
    CYCLE_DURATION = OUT_DURATION + HOLD_DURATION + RETURN_DURATION + NEUTRAL_DURATION
    REQUIRED_CYCLES = 2
    STAGE_DURATION = {
        "static": STATIC_DURATION,
        "elbow": CYCLE_DURATION * REQUIRED_CYCLES,
        "shoulder_front": CYCLE_DURATION * REQUIRED_CYCLES,
        "shoulder_side": CYCLE_DURATION * REQUIRED_CYCLES,
    }
    TOTAL_VALIDATION_DURATION = sum(STAGE_DURATION.values())
    STAGE_LABELS = {
        "static": "靜止穩定度",
        "elbow": "肘關節屈伸",
        "shoulder_front": "肩關節前平舉",
        "shoulder_side": "肩關節側平舉",
    }
    STATIC_STD_LIMIT = 2.5
    STATIC_DRIFT_LIMIT = 4.0
    RETURN_ERROR_LIMIT = 15.0
    ELBOW_MIN_EXCURSION = 15.0
    SHOULDER_MIN_EXCURSION = 8.0
    DIRECTION_AGREEMENT_MIN = 0.75
    PLANE_ACCURACY_MIN = 0.75
    PLANE_SEPARATION_DEG_MIN = 12.0
    AXIS_DOMINANCE_MIN = 1.25
    SAMPLE_GAP_LIMIT = 0.5
    MIN_STAGE_SAMPLES = 50

    def __init__(self):
        self._lock = threading.RLock()
        self.reset()

    def reset(self):
        with getattr(self, "_lock", threading.RLock()):
            self.state = "idle"
            self.current_stage = None
            self.stage_started = None
            self.samples = {}
            self.results = {}
            self.recommendation = None
            self.shoulder_references = None
            self.can_apply = False
            self.applied = False
            self.failure_reasons = []

    def start(self):
        with self._lock:
            self.reset()
            self.state = "ready"
            return self.status()

    def expected_stage(self):
        for stage in self.STAGE_ORDER:
            if stage not in self.results:
                return stage
        return None

    def start_stage(self, stage=None, now=None):
        with self._lock:
            if self.state not in ("ready", "running"):
                raise ValueError("請先開始 IMU 驗證")
            if self.current_stage is not None:
                raise ValueError("目前測試階段尚未完成")
            expected = self.expected_stage()
            stage = stage or expected
            if stage != expected or stage not in self.STAGE_ORDER:
                raise ValueError(f"下一階段必須是 {self.STAGE_LABELS.get(expected, expected)}")
            self.current_stage = stage
            self.stage_started = time.monotonic() if now is None else float(now)
            self.samples[stage] = []
            self.state = "running"
            return self.status(now=now)

    def add_sample(self, frame, now=None):
        with self._lock:
            if self.current_stage is None or self.stage_started is None:
                return
            timestamp = time.monotonic() if now is None else float(now)
            elapsed = timestamp - self.stage_started
            duration = self.STAGE_DURATION[self.current_stage]
            if elapsed < 0.0 or elapsed > duration + 0.5:
                return
            sample = {
                "elapsed": elapsed,
                "elbow_roll": self._number(frame.get("elbow_roll_signed")),
                "elbow_pitch": self._number(frame.get("elbow_pitch_signed")),
                "shoulder": self._number(frame.get("shoulder_angle")),
                "dx": self._number(frame.get("shoulder_dx")),
                "dy": self._number(frame.get("shoulder_dy")),
                "dz": self._number(frame.get("shoulder_dz")),
            }
            self.samples[self.current_stage].append(sample)

    def finish_stage(self, now=None):
        with self._lock:
            if self.current_stage is None or self.stage_started is None:
                raise ValueError("目前沒有進行中的驗證階段")
            timestamp = time.monotonic() if now is None else float(now)
            elapsed = timestamp - self.stage_started
            duration = self.STAGE_DURATION[self.current_stage]
            if elapsed < duration - 0.25:
                raise ValueError(f"本階段尚需 {duration - elapsed:.1f} 秒")
            stage = self.current_stage
            stage_samples = list(self.samples.get(stage, []))
            if stage == "static":
                result = self._analyze_static(stage_samples)
            elif stage == "elbow":
                result = self._analyze_elbow(stage_samples)
            else:
                result = self._analyze_shoulder(stage, stage_samples)
            self.results[stage] = result
            self.current_stage = None
            self.stage_started = None
            self._refresh_overall_result()
            return self.status(now=now)

    def apply(self):
        with self._lock:
            if not self.can_apply or self.recommendation is None:
                raise ValueError("驗證尚未全部通過，不能套用")
            self.applied = True
            self.state = "applied"
            return {
                "elbow_axis": self.recommendation["elbow_axis"],
                "elbow_sign": self.recommendation["elbow_sign"],
                "front_reference": list(self.shoulder_references["front"]),
                "side_reference": list(self.shoulder_references["side"]),
            }

    def status(self, now=None):
        with self._lock:
            timestamp = time.monotonic() if now is None else float(now)
            elapsed = 0.0
            remaining = 0.0
            phase = ""
            cycle = 0
            if self.current_stage is not None and self.stage_started is not None:
                elapsed = max(0.0, timestamp - self.stage_started)
                duration = self.STAGE_DURATION[self.current_stage]
                remaining = max(0.0, duration - elapsed)
                phase, cycle = self._phase_at(self.current_stage, elapsed)
            return {
                "mode": "quick_startup_check",
                "planned_duration": self.TOTAL_VALIDATION_DURATION,
                "required_cycles": self.REQUIRED_CYCLES,
                "state": self.state,
                "current_stage": self.current_stage,
                "current_stage_label": self.STAGE_LABELS.get(self.current_stage, ""),
                "expected_stage": self.expected_stage(),
                "expected_stage_label": self.STAGE_LABELS.get(self.expected_stage(), ""),
                "elapsed": round(elapsed, 2),
                "remaining": round(remaining, 2),
                "phase": phase,
                "cycle": cycle,
                "sample_count": len(self.samples.get(self.current_stage, [])) if self.current_stage else 0,
                "results": self.results,
                "recommendation": self.recommendation,
                "can_apply": self.can_apply,
                "applied": self.applied,
                "passed": bool(self.can_apply or self.applied),
                "failure_reasons": list(self.failure_reasons),
                "motor_interlock_reason": "" if self.applied else "本次初始化尚未完成並套用 IMU 驗證",
            }

    @staticmethod
    def _number(value):
        try:
            value = float(value)
            return value if math.isfinite(value) else 0.0
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _mean(values):
        return statistics.fmean(values) if values else 0.0

    @staticmethod
    def _std(values):
        return statistics.pstdev(values) if len(values) >= 2 else 0.0

    @staticmethod
    def _median(values):
        return statistics.median(values) if values else 0.0

    @staticmethod
    def _vector_mean(samples):
        if not samples:
            return (0.0, 0.0, 0.0)
        return (
            statistics.fmean(s["dx"] for s in samples),
            statistics.fmean(s["dy"] for s in samples),
            statistics.fmean(s["dz"] for s in samples),
        )

    @staticmethod
    def _normalize(vector):
        norm = math.sqrt(sum(v * v for v in vector))
        if norm < 1e-9:
            return (0.0, 0.0, 0.0)
        return tuple(v / norm for v in vector)

    @staticmethod
    def _cosine(a, b):
        return max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))

    @staticmethod
    def _window(samples, start, end):
        return [s for s in samples if start <= s["elapsed"] < end]

    @staticmethod
    def _has_data_gaps(samples):
        if len(samples) < 2:
            return True
        return any(
            b["elapsed"] - a["elapsed"] > IMUValidationSession.SAMPLE_GAP_LIMIT
            for a, b in zip(samples, samples[1:])
        )

    def _static_metric(self, samples, key):
        values = [s[key] for s in samples]
        edge_window = min(2.0, self.STATIC_DURATION / 2.0)
        first = [s[key] for s in samples if s["elapsed"] < edge_window]
        last = [s[key] for s in samples if s["elapsed"] >= self.STATIC_DURATION - edge_window]
        return {
            "std": round(self._std(values), 3),
            "drift": round(abs(self._mean(last) - self._mean(first)), 3),
            "peak_to_peak": round(max(values) - min(values), 3) if values else 0.0,
            "baseline": round(self._mean(last or values), 3),
        }

    def _analyze_static(self, samples):
        enough = len(samples) >= self.MIN_STAGE_SAMPLES and not self._has_data_gaps(samples)
        metrics = {
            "elbow_roll": self._static_metric(samples, "elbow_roll"),
            "elbow_pitch": self._static_metric(samples, "elbow_pitch"),
            "shoulder": self._static_metric(samples, "shoulder"),
        }
        shoulder_ok = (
            metrics["shoulder"]["std"] <= self.STATIC_STD_LIMIT
            and metrics["shoulder"]["drift"] <= self.STATIC_DRIFT_LIMIT
        )
        elbow_axis_available = any(
            metrics[key]["std"] <= self.STATIC_STD_LIMIT
            and metrics[key]["drift"] <= self.STATIC_DRIFT_LIMIT
            for key in ("elbow_roll", "elbow_pitch")
        )
        passed = enough and shoulder_ok and elbow_axis_available
        reasons = []
        if not enough:
            reasons.append("靜止資料不足或通訊中斷")
        if not shoulder_ok:
            reasons.append("肩角靜態雜訊或漂移超過門檻")
        if not elbow_axis_available:
            reasons.append("肘角 roll 與 pitch 均不穩定")
        return {"passed": passed, "metrics": metrics, "reasons": reasons}

    def _cycle_windows(self, samples):
        cycles = []
        for index in range(self.REQUIRED_CYCLES):
            base = index * self.CYCLE_DURATION
            out_end = base + self.OUT_DURATION
            hold_end = out_end + self.HOLD_DURATION
            return_end = hold_end + self.RETURN_DURATION
            neutral_end = return_end + self.NEUTRAL_DURATION
            cycles.append({
                "out": self._window(samples, base, out_end),
                "hold": self._window(samples, out_end, hold_end),
                "return": self._window(samples, hold_end, return_end),
                "neutral": self._window(samples, return_end, neutral_end),
            })
        return cycles

    def _direction_agreement(self, windows, key, sign):
        correct = 0
        active = 0
        for window, expected in windows:
            if len(window) < 4:
                continue
            edge_count = max(1, len(window) // 5)
            start_value = self._mean([sample[key] for sample in window[:edge_count]])
            end_value = self._mean([sample[key] for sample in window[-edge_count:]])
            delta = end_value - start_value
            # A complete phase is one vote. This avoids treating hand tremor or
            # sample-to-sample filter noise as repeated direction reversals.
            if abs(delta) < 2.0:
                continue
            active += 1
            if delta * sign * expected > 0.0:
                correct += 1
        return correct / active if active else 0.0

    def _motion_metrics(self, samples, key, baseline, sign=1.0):
        cycles = self._cycle_windows(samples)
        amplitudes = []
        returns = []
        direction_windows = []
        complete_cycles = 0
        for cycle in cycles:
            if min(len(cycle[name]) for name in ("out", "hold", "return", "neutral")) < 2:
                continue
            complete_cycles += 1
            active_samples = cycle["out"] + cycle["hold"]
            neutral = self._mean([s[key] for s in cycle["neutral"]])
            signed_excursions = [(sample[key] - baseline) * sign for sample in active_samples]
            amplitudes.append(max(0.0, max(signed_excursions, default=0.0)))
            returns.append(abs((neutral - baseline) * sign))
            direction_windows.append((cycle["out"], 1.0))
            direction_windows.append((cycle["return"], -1.0))
        agreement = self._direction_agreement(direction_windows, key, sign)
        return {
            "complete_cycles": complete_cycles,
            "amplitudes": [round(v, 3) for v in amplitudes],
            "return_errors": [round(v, 3) for v in returns],
            "median_amplitude": round(self._median(amplitudes), 3),
            "max_return_error": round(max(returns), 3) if returns else 999.0,
            "direction_agreement": round(agreement, 3),
        }

    def _analyze_elbow(self, samples):
        static = self.results.get("static", {}).get("metrics", {})
        candidates = {}
        for axis in ("roll", "pitch"):
            key = f"elbow_{axis}"
            baseline = static.get(key, {}).get("baseline", 0.0)
            cycles = self._cycle_windows(samples)
            signed_excursions = []
            for cycle in cycles:
                if len(cycle["hold"]) >= 2:
                    signed_excursions.append(
                        self._mean([s[key] for s in cycle["hold"]]) - baseline
                    )
            elbow_sign = 1.0 if self._median(signed_excursions) >= 0.0 else -1.0
            motion = self._motion_metrics(samples, key, baseline, elbow_sign)
            static_std = max(static.get(key, {}).get("std", 0.0), 0.1)
            motion["sign"] = int(elbow_sign)
            motion["score"] = round(motion["median_amplitude"] / static_std, 3)
            candidates[axis] = motion

        ordered = sorted(candidates, key=lambda a: candidates[a]["score"], reverse=True)
        winner, other = ordered[0], ordered[1]
        winner_score = candidates[winner]["score"]
        other_score = max(candidates[other]["score"], 0.001)
        dominance = winner_score / other_score
        metric = candidates[winner]
        static_metric = static.get(f"elbow_{winner}", {})
        passed = (
            len(samples) >= self.MIN_STAGE_SAMPLES
            and not self._has_data_gaps(samples)
            and metric["complete_cycles"] == self.REQUIRED_CYCLES
            and len(metric["amplitudes"]) == self.REQUIRED_CYCLES
            and all(v >= self.ELBOW_MIN_EXCURSION for v in metric["amplitudes"])
            and metric["max_return_error"] <= self.RETURN_ERROR_LIMIT
            and metric["direction_agreement"] >= self.DIRECTION_AGREEMENT_MIN
            and static_metric.get("std", 999.0) <= self.STATIC_STD_LIMIT
            and static_metric.get("drift", 999.0) <= self.STATIC_DRIFT_LIMIT
            and dominance >= self.AXIS_DOMINANCE_MIN
        )
        reasons = []
        if metric["complete_cycles"] != self.REQUIRED_CYCLES:
            reasons.append(f"未完成 {self.REQUIRED_CYCLES} 次肘屈伸")
        if metric["amplitudes"] and any(v < self.ELBOW_MIN_EXCURSION for v in metric["amplitudes"]):
            reasons.append(f"至少一次肘屈曲幅度不足 {self.ELBOW_MIN_EXCURSION:.0f}°")
        if metric["max_return_error"] > self.RETURN_ERROR_LIMIT:
            reasons.append(f"肘關節回零誤差超過 {self.RETURN_ERROR_LIMIT:.0f}°")
        if metric["direction_agreement"] < self.DIRECTION_AGREEMENT_MIN:
            reasons.append(f"肘屈伸方向一致率低於 {self.DIRECTION_AGREEMENT_MIN * 100:.0f}%")
        if dominance < self.AXIS_DOMINANCE_MIN:
            reasons.append("roll／pitch 軸向混雜，請重新對齊 IMU")
        if self._has_data_gaps(samples):
            reasons.append("測試期間 IMU 資料中斷")
        self.recommendation = {
            "elbow_axis": winner,
            "elbow_sign": metric["sign"],
            "axis_dominance": round(dominance, 3),
            "confidence_percent": round(100.0 * winner_score / max(winner_score + candidates[other]["score"], 0.001), 1),
        }
        return {
            "passed": passed,
            "selected_axis": winner,
            "selected_sign": metric["sign"],
            "axis_dominance": round(dominance, 3),
            "candidates": candidates,
            "reasons": reasons,
        }

    def _analyze_shoulder(self, stage, samples):
        baseline = self.results.get("static", {}).get("metrics", {}).get("shoulder", {}).get("baseline", 0.0)
        metrics = self._motion_metrics(samples, "shoulder", baseline, 1.0)
        hold_samples = []
        for cycle in self._cycle_windows(samples):
            hold_samples.extend(cycle["hold"])
        reference = self._normalize(self._vector_mean(hold_samples))
        passed = (
            len(samples) >= self.MIN_STAGE_SAMPLES
            and not self._has_data_gaps(samples)
            and metrics["complete_cycles"] == self.REQUIRED_CYCLES
            and len(metrics["amplitudes"]) == self.REQUIRED_CYCLES
            and all(v >= self.SHOULDER_MIN_EXCURSION for v in metrics["amplitudes"])
            and metrics["max_return_error"] <= self.RETURN_ERROR_LIMIT
            and metrics["direction_agreement"] >= self.DIRECTION_AGREEMENT_MIN
            and math.sqrt(sum(v * v for v in reference)) > 0.9
        )
        reasons = []
        label = "前平舉" if stage == "shoulder_front" else "側平舉"
        if metrics["complete_cycles"] != self.REQUIRED_CYCLES:
            reasons.append(f"未完成 {self.REQUIRED_CYCLES} 次肩關節{label}")
        if metrics["amplitudes"] and any(v < self.SHOULDER_MIN_EXCURSION for v in metrics["amplitudes"]):
            reasons.append(f"至少一次{label}幅度不足 {self.SHOULDER_MIN_EXCURSION:.0f}°")
        if metrics["max_return_error"] > self.RETURN_ERROR_LIMIT:
            reasons.append(f"{label}回零誤差超過 {self.RETURN_ERROR_LIMIT:.0f}°")
        if metrics["direction_agreement"] < self.DIRECTION_AGREEMENT_MIN:
            reasons.append(f"{label}抬舉／下放方向一致率低於 {self.DIRECTION_AGREEMENT_MIN * 100:.0f}%")
        if self._has_data_gaps(samples):
            reasons.append("測試期間 IMU 資料中斷")
        return {
            "passed": passed,
            "metrics": metrics,
            "reference": [round(v, 6) for v in reference],
            "plane_accuracy": None,
            "reasons": reasons,
        }

    def _refresh_overall_result(self):
        self.failure_reasons = []
        for result in self.results.values():
            if not result.get("passed", False):
                self.failure_reasons.extend(result.get("reasons", []))
        if not all(stage in self.results for stage in self.STAGE_ORDER):
            self.state = "ready"
            self.can_apply = False
            return

        front = self._normalize(tuple(self.results["shoulder_front"]["reference"]))
        side = self._normalize(tuple(self.results["shoulder_side"]["reference"]))
        separation = math.degrees(math.acos(self._cosine(front, side)))
        front_accuracy = self._plane_accuracy("shoulder_front", front, side, expected="front")
        side_accuracy = self._plane_accuracy("shoulder_side", front, side, expected="side")
        self.results["shoulder_front"]["plane_accuracy"] = round(front_accuracy, 3)
        self.results["shoulder_side"]["plane_accuracy"] = round(side_accuracy, 3)
        plane_ok = (
            separation >= self.PLANE_SEPARATION_DEG_MIN
            and front_accuracy >= self.PLANE_ACCURACY_MIN
            and side_accuracy >= self.PLANE_ACCURACY_MIN
        )
        if not plane_ok:
            self.results["shoulder_front"]["passed"] = False
            self.results["shoulder_side"]["passed"] = False
            self.failure_reasons.append("前平舉與側平舉特徵無法可靠區分")
        self.shoulder_references = {"front": front, "side": side}
        if self.recommendation is not None:
            self.recommendation["shoulder_plane_separation_deg"] = round(separation, 2)
            self.recommendation["front_accuracy"] = round(front_accuracy, 3)
            self.recommendation["side_accuracy"] = round(side_accuracy, 3)
        self.can_apply = all(self.results[stage].get("passed", False) for stage in self.STAGE_ORDER)
        self.state = "complete" if self.can_apply else "failed"

    def _plane_accuracy(self, stage, front, side, expected):
        correct = 0
        total = 0
        for cycle in self._cycle_windows(self.samples.get(stage, [])):
            for sample in cycle["hold"]:
                vector = self._normalize((sample["dx"], sample["dy"], sample["dz"]))
                if vector == (0.0, 0.0, 0.0):
                    continue
                predicted = "front" if self._cosine(vector, front) >= self._cosine(vector, side) else "side"
                total += 1
                correct += int(predicted == expected)
        return correct / total if total else 0.0

    def _phase_at(self, stage, elapsed):
        if stage == "static":
            return "保持自然下垂、肘伸直並完全靜止", 0
        cycle = min(int(elapsed // self.CYCLE_DURATION) + 1, self.REQUIRED_CYCLES)
        phase_time = elapsed % self.CYCLE_DURATION
        joint = "肘關節" if stage == "elbow" else "手臂"
        out_end = self.OUT_DURATION
        hold_end = out_end + self.HOLD_DURATION
        return_end = hold_end + self.RETURN_DURATION
        if phase_time < out_end:
            action = "緩慢屈曲" if stage == "elbow" else "緩慢抬舉"
        elif phase_time < hold_end:
            action = "保持屈曲" if stage == "elbow" else "保持抬舉"
        elif phase_time < return_end:
            action = "緩慢伸展" if stage == "elbow" else "緩慢下放"
        else:
            action = "保持回零"
        return f"第 {cycle}/{self.REQUIRED_CYCLES} 次：{joint}{action}", cycle
