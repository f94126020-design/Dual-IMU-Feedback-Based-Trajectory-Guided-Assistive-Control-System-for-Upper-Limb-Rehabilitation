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
    REQUIRED_CYCLES = 2
    OUT_DURATION = 1.5
    HOLD_DURATION = 0.75
    RETURN_DURATION = 1.5
    NEUTRAL_DURATION = 1.25
    CYCLE_DURATION = OUT_DURATION + HOLD_DURATION + RETURN_DURATION + NEUTRAL_DURATION
    # Keep these explicit: comprehensions inside a class body cannot resolve
    # sibling class attributes on every supported Python version.
    STAGE_DURATION = {
        "elbow_flexion": 10.0,
        "elbow_extension": 10.0,
        "shoulder_front": 10.0,
        "shoulder_side": 10.0,
    }
    SAMPLE_GAP_LIMIT = 0.5
    MIN_HOLD_SAMPLES = 4
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
            if elapsed < 0.0 or elapsed > self.STAGE_DURATION[self.current_stage] + 0.5:
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
            duration = self.STAGE_DURATION[self.current_stage]
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
                remaining = max(0.0, self.STAGE_DURATION[self.current_stage] - elapsed)
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

    @staticmethod
    def _window(samples, start, end):
        return [sample for sample in samples if start <= sample["elapsed"] < end]

    def _cycle_windows(self, samples):
        cycles = []
        for index in range(self.REQUIRED_CYCLES):
            base = index * self.CYCLE_DURATION
            out_end = base + self.OUT_DURATION
            hold_end = out_end + self.HOLD_DURATION
            return_end = hold_end + self.RETURN_DURATION
            cycles.append({
                "out": self._window(samples, base, out_end),
                "hold": self._window(samples, out_end, hold_end),
                "return": self._window(samples, hold_end, return_end),
                "neutral": self._window(samples, return_end, base + self.CYCLE_DURATION),
            })
        return cycles

    def _hold_and_neutral(self, samples):
        hold = []
        neutral = []
        for cycle in self._cycle_windows(samples):
            hold.extend(cycle["hold"])
            neutral.extend(cycle["neutral"])
        return hold, neutral

    def _technical_warnings(self, samples, hold, label):
        warnings = []
        if len(hold) < self.MIN_HOLD_SAMPLES:
            warnings.append(f"{label}保持區段資料不足")
        if self._has_data_gaps(samples):
            warnings.append(f"{label}期間 IMU 資料有中斷")
        return warnings

    def _analyze_elbow_flexion(self, samples):
        hold, neutral = self._hold_and_neutral(samples)
        candidates = {}
        for axis in ("roll", "pitch"):
            baseline = self._median([sample[f"elbow_{axis}"] for sample in neutral])
            flex_hold = self._median([sample[f"elbow_{axis}"] for sample in hold])
            excursion = flex_hold - baseline
            candidates[axis] = {
                "baseline": round(baseline, 3),
                "signed_excursion": round(excursion, 3),
                "absolute_excursion": round(abs(excursion), 3),
            }
        axis = max(candidates, key=lambda name: candidates[name]["absolute_excursion"])
        sign = 1 if candidates[axis]["signed_excursion"] >= 0.0 else -1
        baseline = candidates[axis]["baseline"]
        mapped_hold = [sign * (sample[f"elbow_{axis}"] - baseline) for sample in hold]
        flexion_max = self._percentile(mapped_hold, 0.90)
        other = "pitch" if axis == "roll" else "roll"
        dominance = candidates[axis]["absolute_excursion"] / max(candidates[other]["absolute_excursion"], 0.1)
        warnings = self._technical_warnings(samples, hold, "肘屈曲")
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
        hold, neutral = self._hold_and_neutral(samples)
        extension_angle = self._percentile(self._mapped_elbow_values(hold), 0.10)
        start_90_angle = self._median(self._mapped_elbow_values(neutral))
        warnings = self._technical_warnings(samples, hold, "肘伸展")
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
        hold, neutral = self._hold_and_neutral(samples)
        maximum = self._percentile([sample["shoulder"] for sample in hold], 0.90)
        return_angle = self._median([sample["shoulder"] for sample in neutral])
        vector = self._normalize((
            self._median([sample["dx"] for sample in hold]),
            self._median([sample["dy"] for sample in hold]),
            self._median([sample["dz"] for sample in hold]),
        ))
        label = "前平舉" if stage == "shoulder_front" else "側平舉"
        warnings = self._technical_warnings(samples, hold, label)
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
        cycle = min(int(elapsed // self.CYCLE_DURATION) + 1, self.REQUIRED_CYCLES)
        phase_time = elapsed % self.CYCLE_DURATION
        if stage == "elbow_flexion":
            actions = ("屈曲到最大", "保持最大屈曲", "伸直回到下垂", "保持肘伸直")
        elif stage == "elbow_extension":
            actions = ("從 90° 向下伸直", "保持打直或微過伸", "彎回 90°", "保持 90°")
        elif stage == "shoulder_front":
            actions = ("向前抬到最大", "保持最大前舉", "沿原路放下", "保持自然下垂")
        else:
            actions = ("向側面抬到最大", "保持最大側舉", "沿原路放下", "保持自然下垂")
        if phase_time < self.OUT_DURATION:
            action = actions[0]
        elif phase_time < self.OUT_DURATION + self.HOLD_DURATION:
            action = actions[1]
        elif phase_time < self.OUT_DURATION + self.HOLD_DURATION + self.RETURN_DURATION:
            action = actions[2]
        else:
            action = actions[3]
        return f"第 {cycle}/{self.REQUIRED_CYCLES} 次：{action}", cycle
