"""Patient-specific, task-oriented upper-limb ROM calibration.

This workflow records repeatable session-relative ranges. It intentionally does
not grade the patient PASS/FAIL and does not claim clinical goniometer accuracy.
"""

from __future__ import annotations

import math
import statistics
import threading
import time


class ROMCalibrationSession:
    STAGE_ORDER = (
        "elbow_flexion",
        "elbow_extension",
        "shoulder_front",
        "shoulder_side",
    )
    STAGE_LABELS = {
        "elbow_flexion": "手臂下垂－最大肘屈曲",
        "elbow_extension": "肘 90°－三頭肌伸展至打直",
        "shoulder_front": "最大前平舉",
        "shoulder_side": "最大側平舉",
    }
    DEFAULT_STAGE_DURATION = 10.0
    MIN_STAGE_DURATION = 5.0
    MAX_STAGE_DURATION = 30.0
    SAMPLE_GAP_LIMIT = 0.5
    MIN_STAGE_SAMPLES = 40
    TRAINING_FRACTION = 0.90

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
            self.mapping = None
            self.rom = None
            self.applied = False
            self.warnings = []
            self.stage_duration_seconds = self.DEFAULT_STAGE_DURATION

    def start(self, duration_seconds=None):
        with self._lock:
            self.reset()
            if duration_seconds is not None:
                try:
                    requested = float(duration_seconds)
                except (TypeError, ValueError):
                    requested = self.DEFAULT_STAGE_DURATION
                self.stage_duration_seconds = max(
                    self.MIN_STAGE_DURATION,
                    min(self.MAX_STAGE_DURATION, requested),
                )
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
                raise ValueError("請先開始病患 ROM 校正")
            if self.current_stage is not None:
                raise ValueError("目前 ROM 階段尚未完成")
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
            if elapsed < 0.0 or elapsed > self.stage_duration_seconds + 0.5:
                return
            self.samples[self.current_stage].append({
                "elapsed": elapsed,
                "elbow_roll": self._number(frame.get("elbow_roll_signed")),
                "elbow_pitch": self._number(frame.get("elbow_pitch_signed")),
                "shoulder": self._number(frame.get("shoulder_angle")),
                "dx": self._number(frame.get("shoulder_dx")),
                "dy": self._number(frame.get("shoulder_dy")),
                "dz": self._number(frame.get("shoulder_dz")),
            })

    def finish_stage(self, now=None):
        with self._lock:
            if self.current_stage is None or self.stage_started is None:
                raise ValueError("目前沒有進行中的 ROM 階段")
            timestamp = time.monotonic() if now is None else float(now)
            elapsed = timestamp - self.stage_started
            duration = self.stage_duration_seconds
            if elapsed < duration - 0.25:
                raise ValueError(f"本階段尚需 {duration - elapsed:.1f} 秒")
            stage = self.current_stage
            samples = list(self.samples.get(stage, []))
            if stage == "elbow_flexion":
                result = self._analyze_elbow_flexion(samples)
            elif stage == "elbow_extension":
                result = self._analyze_elbow_extension(samples)
            else:
                result = self._analyze_shoulder(stage, samples)
            self.results[stage] = result
            self.current_stage = None
            self.stage_started = None
            self._refresh_summary()
            return self.status(now=now)

    def apply(self):
        with self._lock:
            if not self.ready_to_apply or self.mapping is None or self.rom is None:
                raise ValueError("四個 ROM 動作尚未完整記錄")
            self.applied = True
            self.state = "applied"
            return {
                "elbow_axis": self.mapping["elbow_axis"],
                "elbow_sign": self.mapping["elbow_sign"],
                "front_reference": list(self.mapping["front_reference"]),
                "side_reference": list(self.mapping["side_reference"]),
                "rom": dict(self.rom),
            }

    @property
    def ready_to_apply(self):
        return all(stage in self.results for stage in self.STAGE_ORDER) and self.rom is not None

    def status(self, now=None):
        with self._lock:
            timestamp = time.monotonic() if now is None else float(now)
            elapsed = 0.0
            remaining = 0.0
            phase = ""
            cycle = 0
            if self.current_stage is not None and self.stage_started is not None:
                elapsed = max(0.0, timestamp - self.stage_started)
                remaining = max(0.0, self.stage_duration_seconds - elapsed)
                phase, cycle = self._phase_at(self.current_stage, elapsed)
            return {
                "mode": "patient_rom",
                "state": self.state,
                "current_stage": self.current_stage,
                "current_stage_label": self.STAGE_LABELS.get(self.current_stage, ""),
                "expected_stage": self.expected_stage(),
                "expected_stage_label": self.STAGE_LABELS.get(self.expected_stage(), ""),
                "elapsed": round(elapsed, 2),
                "remaining": round(remaining, 2),
                "stage_duration_seconds": self.stage_duration_seconds,
                "phase": phase,
                "cycle": cycle,
                "sample_count": len(self.samples.get(self.current_stage, [])) if self.current_stage else 0,
                "results": self.results,
                "mapping": self.mapping,
                "rom": self.rom,
                "ready_to_apply": self.ready_to_apply,
                "applied": self.applied,
                "warnings": list(self.warnings),
            }

    @staticmethod
    def _number(value):
        try:
            value = float(value)
            return value if math.isfinite(value) else 0.0
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _median(values):
        return statistics.median(values) if values else 0.0

    @staticmethod
    def _percentile(values, fraction):
        if not values:
            return 0.0
        ordered = sorted(values)
        index = max(0, min(len(ordered) - 1, round((len(ordered) - 1) * fraction)))
        return ordered[index]

    @staticmethod
    def _normalize(vector):
        norm = math.sqrt(sum(v * v for v in vector))
        return tuple(v / norm for v in vector) if norm > 1e-9 else (0.0, 0.0, 0.0)

    @staticmethod
    def _has_data_gaps(samples):
        if len(samples) < 2:
            return True
        return any(
            b["elapsed"] - a["elapsed"] > ROMCalibrationSession.SAMPLE_GAP_LIMIT
            for a, b in zip(samples, samples[1:])
        )

    def _free_motion_warnings(self, samples, label):
        warnings = []
        if len(samples) < self.MIN_STAGE_SAMPLES:
            warnings.append(
                f"{label}有效樣本不足（{len(samples)}/{self.MIN_STAGE_SAMPLES}）"
            )
        if self._has_data_gaps(samples):
            warnings.append(f"{label}期間 IMU 資料有中斷")
        return warnings

    def _analyze_elbow_flexion(self, samples):
        candidates = {}
        for axis in ("roll", "pitch"):
            values = [sample[f"elbow_{axis}"] for sample in samples]
            initial = [
                sample[f"elbow_{axis}"] for sample in samples
                if sample["elapsed"] <= 1.0
            ]
            baseline = self._median(initial or values)
            low = self._percentile(values, 0.05)
            high = self._percentile(values, 0.95)
            positive_excursion = high - baseline
            negative_excursion = baseline - low
            sign = 1 if positive_excursion >= negative_excursion else -1
            excursion = max(positive_excursion, negative_excursion)
            candidates[axis] = {
                "baseline": round(baseline, 3),
                "sign": sign,
                "low_percentile": round(low, 3),
                "high_percentile": round(high, 3),
                "absolute_excursion": round(abs(excursion), 3),
            }
        axis = max(candidates, key=lambda name: candidates[name]["absolute_excursion"])
        sign = candidates[axis]["sign"]
        baseline = candidates[axis]["baseline"]
        mapped_values = [sign * (sample[f"elbow_{axis}"] - baseline) for sample in samples]
        flexion_max = self._percentile(mapped_values, 0.95)
        other = "pitch" if axis == "roll" else "roll"
        dominance = candidates[axis]["absolute_excursion"] / max(candidates[other]["absolute_excursion"], 0.1)
        warnings = self._free_motion_warnings(samples, "肘屈曲")
        if flexion_max < 5.0:
            warnings.append("記錄到的肘屈曲範圍很小，請確認 IMU 固定位置")
        self.mapping = {
            "elbow_axis": axis,
            "elbow_sign": sign,
            "elbow_baseline_raw": baseline,
            "axis_dominance": round(dominance, 3),
        }
        return {
            "recorded": True,
            "elbow_axis": axis,
            "elbow_sign": sign,
            "flexion_max_deg": round(flexion_max, 2),
            "candidates": candidates,
            "warnings": warnings,
        }

    def _mapped_elbow_values(self, samples):
        if self.mapping is None:
            return []
        axis = self.mapping["elbow_axis"]
        sign = self.mapping["elbow_sign"]
        baseline = self.mapping["elbow_baseline_raw"]
        return [sign * (sample[f"elbow_{axis}"] - baseline) for sample in samples]

    def _analyze_elbow_extension(self, samples):
        mapped_values = self._mapped_elbow_values(samples)
        extension_angle = self._percentile(mapped_values, 0.05)
        start_90_angle = self._percentile(mapped_values, 0.95)
        warnings = self._free_motion_warnings(samples, "肘伸展")
        if start_90_angle - extension_angle < 5.0:
            warnings.append("90° 到伸直位置的變化很小，請確認起始姿勢")
        return {
            "recorded": True,
            "extension_angle_deg": round(extension_angle, 2),
            "start_90_angle_deg": round(start_90_angle, 2),
            "extension_excursion_deg": round(start_90_angle - extension_angle, 2),
            "warnings": warnings,
        }

    def _analyze_shoulder(self, stage, samples):
        shoulder_values = [sample["shoulder"] for sample in samples]
        maximum = self._percentile(shoulder_values, 0.95)
        return_angle = self._percentile(shoulder_values, 0.05)
        high_threshold = self._percentile(shoulder_values, 0.90)
        high_samples = [sample for sample in samples if sample["shoulder"] >= high_threshold]
        vector = self._normalize((
            self._median([sample["dx"] for sample in high_samples]),
            self._median([sample["dy"] for sample in high_samples]),
            self._median([sample["dz"] for sample in high_samples]),
        ))
        label = "前平舉" if stage == "shoulder_front" else "側平舉"
        warnings = self._free_motion_warnings(samples, label)
        if maximum - return_angle < 5.0:
            warnings.append(f"記錄到的{label}範圍很小")
        return {
            "recorded": True,
            "maximum_deg": round(maximum, 2),
            "return_angle_deg": round(return_angle, 2),
            "reference": [round(value, 6) for value in vector],
            "warnings": warnings,
        }

    def _refresh_summary(self):
        self.warnings = []
        for result in self.results.values():
            self.warnings.extend(result.get("warnings", []))
        if not all(stage in self.results for stage in self.STAGE_ORDER):
            self.state = "ready"
            return

        flex = self.results["elbow_flexion"]["flexion_max_deg"]
        extension = self.results["elbow_extension"]["extension_angle_deg"]
        elbow_min = min(extension, flex)
        elbow_max = max(extension, flex)
        elbow_span = max(elbow_max - elbow_min, 0.0)
        front_max = max(0.0, self.results["shoulder_front"]["maximum_deg"])
        side_max = max(0.0, self.results["shoulder_side"]["maximum_deg"])
        self.rom = {
            "elbow_extension_deg": round(elbow_min, 2),
            "elbow_flexion_deg": round(elbow_max, 2),
            "elbow_flexion_target_deg": round(elbow_min + self.TRAINING_FRACTION * elbow_span, 2),
            "elbow_extension_target_deg": round(elbow_min + (1.0 - self.TRAINING_FRACTION) * elbow_span, 2),
            "shoulder_front_max_deg": round(front_max, 2),
            "shoulder_front_target_deg": round(self.TRAINING_FRACTION * front_max, 2),
            "shoulder_side_max_deg": round(side_max, 2),
            "shoulder_side_target_deg": round(self.TRAINING_FRACTION * side_max, 2),
            "training_fraction": self.TRAINING_FRACTION,
        }
        self.mapping.update({
            "front_reference": tuple(self.results["shoulder_front"]["reference"]),
            "side_reference": tuple(self.results["shoulder_side"]["reference"]),
        })
        self.state = "complete"

    def _phase_at(self, stage, elapsed):
        if stage == "elbow_flexion":
            action = "上臂保持下垂，自由重複肘伸直到最大屈曲"
        elif stage == "elbow_extension":
            action = "自由重複肘約 90° 到完全伸直"
        elif stage == "shoulder_front":
            action = "自由重複自然下垂到最大前平舉"
        else:
            action = "自由重複自然下垂到最大側平舉"
        return f"自由活動：{action}；系統自動搜尋最大／最小角度", 0
