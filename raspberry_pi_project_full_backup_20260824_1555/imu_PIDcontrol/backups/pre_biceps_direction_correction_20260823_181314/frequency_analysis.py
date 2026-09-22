import csv
import json
import math
import os
import threading
import time
from datetime import datetime

import numpy as np


class FrequencyIdentificationSession:
    """Safe joint-level excitation, capture, and NumPy-only frequency analysis."""

    SIGNAL_TYPES = ("chirp", "prbs", "sine")

    def __init__(self):
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        with getattr(self, "lock", threading.RLock()):
            self.state = "idle"
            self.reason = ""
            self.config = {}
            self.samples = []
            self.started_at = None
            self.finished_at = None
            self.prbs_values = np.array([], dtype=float)
            self.result = None
            self.output_directory = None
            self.csv_path = None
            self.json_path = None
            self.png_path = None

    @staticmethod
    def _finite(value, default):
        try:
            result = float(value)
        except (TypeError, ValueError):
            result = float(default)
        if not math.isfinite(result):
            result = float(default)
        return result

    def validate_config(self, data):
        signal_type = str(data.get("signal_type", "chirp")).lower()
        if signal_type not in self.SIGNAL_TYPES:
            raise ValueError("訊號類型只允許 chirp、prbs 或 sine。")
        joint = str(data.get("target_joint", "elbow")).lower()
        if joint not in ("elbow", "shoulder"):
            raise ValueError("測試關節只允許 elbow 或 shoulder。")

        duration = min(max(self._finite(data.get("duration"), 60.0), 10.0), 180.0)
        amplitude = min(max(abs(self._finite(data.get("amplitude"), 12.0)), 1.0), 30.0)
        f_start = min(max(self._finite(data.get("f_start"), 0.1), 0.05), 5.0)
        f_end = min(max(self._finite(data.get("f_end"), 3.0), 0.05), 5.0)
        if signal_type == "chirp" and f_end <= f_start:
            raise ValueError("Chirp 結束頻率必須大於起始頻率。")
        prbs_rate = min(max(self._finite(data.get("prbs_rate"), 2.0), 0.2), 10.0)
        safe_min = self._finite(data.get("safe_min_angle"), -10.0)
        safe_max = self._finite(data.get("safe_max_angle"), 120.0)
        if safe_max - safe_min < 10.0:
            raise ValueError("安全角度上下限至少需相差 10°。")
        if safe_min < -30.0 or safe_max > 180.0:
            raise ValueError("安全角度範圍必須位於 -30° 到 180°。")

        return {
            "signal_type": signal_type,
            "target_joint": joint,
            "duration": duration,
            "amplitude": amplitude,
            "f_start": f_start,
            "f_end": f_end,
            "prbs_rate": prbs_rate,
            "safe_min_angle": safe_min,
            "safe_max_angle": safe_max,
            "coherence_threshold": 0.5,
            "fade_seconds": min(1.0, duration * 0.05),
        }

    def start(self, data, initial_angle, output_root="frequency_data", now=None):
        config = self.validate_config(data)
        initial_angle = self._finite(initial_angle, 0.0)
        if not config["safe_min_angle"] <= initial_angle <= config["safe_max_angle"]:
            raise ValueError(
                f"目前角度 {initial_angle:.1f}° 不在安全範圍 "
                f"{config['safe_min_angle']:.1f}° 到 {config['safe_max_angle']:.1f}°。"
            )

        with self.lock:
            if self.state == "running":
                raise ValueError("系統識別測試已在執行。")
            self.reset()
            self.config = config
            self.started_at = time.monotonic() if now is None else float(now)
            self.state = "running"
            count = int(math.ceil(config["duration"] * config["prbs_rate"])) + 2
            rng = np.random.default_rng(20260811)
            self.prbs_values = rng.choice((-1.0, 1.0), size=count)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_directory = os.path.abspath(
                os.path.join(output_root, f"{stamp}_{config['target_joint']}_{config['signal_type']}")
            )
            return self.status(now=self.started_at)

    def elapsed(self, now=None):
        if self.started_at is None:
            return 0.0
        current = time.monotonic() if now is None else float(now)
        return max(0.0, current - self.started_at)

    def command(self, now=None):
        with self.lock:
            if self.state != "running":
                return 0.0
            t = self.elapsed(now)
            duration = self.config["duration"]
            if t >= duration:
                return 0.0
            fade = self.config["fade_seconds"]
            envelope = min(1.0, t / max(fade, 1e-6), (duration - t) / max(fade, 1e-6))
            envelope = max(0.0, envelope)
            amplitude = self.config["amplitude"]
            signal_type = self.config["signal_type"]

            if signal_type == "prbs":
                index = min(int(t * self.config["prbs_rate"]), len(self.prbs_values) - 1)
                value = self.prbs_values[index]
            else:
                f0 = self.config["f_start"]
                if signal_type == "chirp":
                    slope = (self.config["f_end"] - f0) / duration
                    phase = 2.0 * math.pi * (f0 * t + 0.5 * slope * t * t)
                else:
                    phase = 2.0 * math.pi * f0 * t
                value = math.sin(phase)
            return float(amplitude * envelope * value)

    def add_sample(self, frame, requested_command, effective_command, pwm, now=None):
        with self.lock:
            if self.state != "running":
                return None
            current = time.monotonic() if now is None else float(now)
            elapsed = self.elapsed(current)
            angle = self._finite(frame.get("measured_angle"), 0.0)
            sample = {
                "elapsed": elapsed,
                "wall_time": time.time(),
                "frame_time": self._finite(frame.get("time"), 0.0),
                "requested_command_pwm": self._finite(requested_command, 0.0),
                "command_pwm": self._finite(effective_command, 0.0),
                "measured_angle": angle,
                "measured_angle_raw": self._finite(frame.get("measured_angle_raw"), angle),
                "target_joint": self.config["target_joint"],
                "pwm_biceps": int(getattr(pwm, "biceps", 0)),
                "pwm_triceps": int(getattr(pwm, "triceps", 0)),
                "pwm_deltoid": int(getattr(pwm, "deltoid", 0)),
            }
            self.samples.append(sample)
            if not self.config["safe_min_angle"] <= angle <= self.config["safe_max_angle"]:
                self.state = "aborted"
                self.reason = f"關節角度 {angle:.1f}° 超出安全範圍。"
                self.finished_at = current
                return "aborted"
            if elapsed >= self.config["duration"]:
                self.state = "completed"
                self.reason = "測試時間完成。"
                self.finished_at = current
                return "completed"
            return None

    def stop(self, reason="使用者停止測試。", aborted=True, now=None):
        with self.lock:
            if self.state == "running":
                self.state = "aborted" if aborted else "completed"
                self.reason = str(reason)
                self.finished_at = time.monotonic() if now is None else float(now)
            return self.status(now=now)

    def status(self, now=None):
        with self.lock:
            elapsed = self.elapsed(now)
            duration = float(self.config.get("duration", 0.0))
            summary = None
            if self.result:
                summary = self.result.get("summary")
            return {
                "state": self.state,
                "reason": self.reason,
                "config": dict(self.config),
                "elapsed": elapsed,
                "remaining": max(0.0, duration - elapsed) if self.state == "running" else 0.0,
                "sample_count": len(self.samples),
                "current_command": self.command(now) if self.state == "running" else 0.0,
                "summary": summary,
                "csv_path": self.csv_path,
                "json_path": self.json_path,
                "png_path": self.png_path,
                "human_test_allowed": False,
            }

    @staticmethod
    def _welch_frf(u, y, fs):
        n = len(u)
        max_segment = min(2048, max(64, n // 2))
        nperseg = 2 ** int(math.floor(math.log2(max_segment)))
        nperseg = min(nperseg, n)
        if nperseg < 64:
            raise ValueError("有效樣本不足，無法計算頻率響應。")
        step = max(1, nperseg // 2)
        window = np.hanning(nperseg)
        suu = None
        syy = None
        suy = None
        segments = 0
        for start in range(0, n - nperseg + 1, step):
            us = u[start:start + nperseg]
            ys = y[start:start + nperseg]
            us = (us - np.mean(us)) * window
            ys = (ys - np.mean(ys)) * window
            U = np.fft.rfft(us)
            Y = np.fft.rfft(ys)
            this_suu = np.conj(U) * U
            this_syy = np.conj(Y) * Y
            this_suy = np.conj(U) * Y
            suu = this_suu if suu is None else suu + this_suu
            syy = this_syy if syy is None else syy + this_syy
            suy = this_suy if suy is None else suy + this_suy
            segments += 1
        suu /= segments
        syy /= segments
        suy /= segments
        eps = 1e-12
        h1 = suy / (suu + eps)
        coherence = np.abs(suy) ** 2 / (np.real(suu) * np.real(syy) + eps)
        frequency = np.fft.rfftfreq(nperseg, d=1.0 / fs)
        return frequency, h1, np.clip(np.real(coherence), 0.0, 1.0), nperseg, segments

    @staticmethod
    def _fit_arx(u, y, dt):
        n = len(y)
        split = max(10, int(n * 0.7))
        best = None
        # Linux scheduling, serial transport, the ESP32 ramp and the mechanics
        # create an unknown integer-sample delay. Select 0..5 samples by the
        # training one-step residual instead of assuming a delay beforehand.
        for delay in range(6):
            rows = []
            target = []
            first_k = max(2, delay + 1)
            for k in range(first_k, split):
                rows.append([y[k - 1], y[k - 2], u[k - delay], u[k - delay - 1], 1.0])
                target.append(y[k])
            matrix = np.asarray(rows)
            target_array = np.asarray(target)
            theta_candidate, _, _, _ = np.linalg.lstsq(matrix, target_array, rcond=None)
            residual = float(np.mean((matrix @ theta_candidate - target_array) ** 2))
            if best is None or residual < best[0]:
                best = (residual, delay, theta_candidate)
        _, delay, theta = best
        a1, a2, b1, b2, bias = [float(v) for v in theta]
        prediction = np.zeros(n - split)
        history = [float(y[split - 2]), float(y[split - 1])]
        for index, k in enumerate(range(split, n)):
            input_index = k - delay
            value = (
                a1 * history[-1]
                + a2 * history[-2]
                + b1 * u[input_index]
                + b2 * u[input_index - 1]
                + bias
            )
            prediction[index] = value
            history.append(value)
        actual = y[split:]
        denominator = np.linalg.norm(actual - np.mean(actual))
        fit_percent = 100.0 * (1.0 - np.linalg.norm(actual - prediction) / max(denominator, 1e-12))

        poles_z = np.roots([1.0, -a1, -a2])
        poles_s = np.log(poles_z.astype(complex)) / dt
        stable = bool(np.all(np.abs(poles_z) < 1.0))
        dominant = poles_s[np.argmax(np.real(poles_s))]
        natural_frequency = float(abs(dominant))
        damping_ratio = float(-np.real(dominant) / max(abs(dominant), 1e-12))
        dc_denominator = 1.0 - a1 - a2
        dc_gain = float((b1 + b2) / dc_denominator) if abs(dc_denominator) > 1e-9 else None
        return {
            "equation": "y[k]=a1*y[k-1]+a2*y[k-2]+b1*u[k-delay]+b2*u[k-delay-1]+bias",
            "input_delay_samples": int(delay),
            "input_delay_seconds": float(delay * dt),
            "a1": a1,
            "a2": a2,
            "b1": b1,
            "b2": b2,
            "bias": bias,
            "validation_fit_percent": float(fit_percent),
            "stable": stable,
            "poles_z": [[float(np.real(p)), float(np.imag(p))] for p in poles_z],
            "equivalent_natural_frequency_rad_s": natural_frequency,
            "equivalent_natural_frequency_hz": natural_frequency / (2.0 * math.pi),
            "equivalent_damping_ratio": damping_ratio,
            "dc_gain_deg_per_pwm": dc_gain,
        }

    def _write_csv(self):
        os.makedirs(self.output_directory, exist_ok=True)
        self.csv_path = os.path.join(self.output_directory, "raw_frequency_data.csv")
        fields = list(self.samples[0].keys())
        with open(self.csv_path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.samples)

    def save_raw(self):
        with self.lock:
            if not self.samples:
                return None
            self._write_csv()
            return self.csv_path

    def analyze(self):
        with self.lock:
            if self.state == "running":
                raise ValueError("請先完成或停止測試，再執行分析。")
            if len(self.samples) < 128:
                raise ValueError("有效樣本不足 128 筆，無法進行頻域分析。")
            samples = list(self.samples)
            config = dict(self.config)

        t = np.asarray([row["elapsed"] for row in samples], dtype=float)
        u = np.asarray([row["command_pwm"] for row in samples], dtype=float)
        y = np.asarray([row["measured_angle"] for row in samples], dtype=float)
        valid = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
        t, u, y = t[valid], u[valid], y[valid]
        order = np.argsort(t)
        t, u, y = t[order], u[order], y[order]
        unique = np.concatenate(([True], np.diff(t) > 1e-6))
        t, u, y = t[unique], u[unique], y[unique]
        dt_values = np.diff(t)
        dt = float(np.median(dt_values))
        if not 0.002 <= dt <= 0.1:
            raise ValueError(f"取樣間隔中位數 {dt:.4f}s 不適合分析。")
        uniform_t = np.arange(t[0], t[-1], dt)
        uniform_u = np.interp(uniform_t, t, u)
        uniform_y = np.interp(uniform_t, t, y)
        uniform_u -= np.mean(uniform_u)
        trend = np.polyval(np.polyfit(uniform_t, uniform_y, 1), uniform_t)
        uniform_y -= trend
        fs = 1.0 / dt

        frequency, h1, coherence, nperseg, segments = self._welch_frf(uniform_u, uniform_y, fs)
        gain_db = 20.0 * np.log10(np.maximum(np.abs(h1), 1e-12))
        phase_deg = np.degrees(np.unwrap(np.angle(h1)))
        analysis_max = max(config["f_start"], config["f_end"] if config["signal_type"] == "chirp" else config["f_start"] * 1.5)
        band = (frequency >= max(0.02, config["f_start"] * 0.5)) & (frequency <= min(5.0, analysis_max * 1.1))
        trusted = band & (coherence >= config["coherence_threshold"])
        trusted_indices = np.flatnonzero(trusted)

        resonance_frequency = None
        bandwidth_frequency = None
        delay_seconds = None
        if len(trusted_indices) >= 3:
            resonance_index = trusted_indices[np.argmax(gain_db[trusted_indices])]
            resonance_frequency = float(frequency[resonance_index])
            low_reference = float(np.median(gain_db[trusted_indices[: min(3, len(trusted_indices))]]))
            below = trusted_indices[gain_db[trusted_indices] <= low_reference - 3.0]
            if len(below):
                bandwidth_frequency = float(frequency[below[0]])
            omega = 2.0 * math.pi * frequency[trusted_indices]
            phase_rad = np.unwrap(np.angle(h1[trusted_indices]))
            slope, _ = np.polyfit(omega, phase_rad, 1)
            delay_seconds = float(max(0.0, -slope))

        arx = self._fit_arx(uniform_u, uniform_y, dt)
        timing_jitter = float(np.std(dt_values))
        result = {
            "summary": {
                "sample_count": int(len(t)),
                "effective_sample_rate_hz": float(fs),
                "median_dt_seconds": dt,
                "timing_jitter_std_seconds": timing_jitter,
                "welch_segment_length": int(nperseg),
                "welch_segment_count": int(segments),
                "trusted_frequency_bins": int(np.sum(trusted)),
                "coherence_threshold": config["coherence_threshold"],
                "resonance_frequency_hz": resonance_frequency,
                "bandwidth_3db_hz": bandwidth_frequency,
                "equivalent_phase_delay_seconds": delay_seconds,
                "warning": "頻域結果僅代表本次固定方式與負載；無 encoder 時不可解讀為馬達軸頻率響應。",
            },
            "arx_model": arx,
            "frequency_hz": frequency[band].astype(float).tolist(),
            "gain_db": gain_db[band].astype(float).tolist(),
            "phase_deg": phase_deg[band].astype(float).tolist(),
            "coherence": coherence[band].astype(float).tolist(),
        }

        with self.lock:
            self.result = result
            self.state = "analyzed"
            self._write_csv()
            self.json_path = os.path.join(self.output_directory, "frequency_analysis.json")
            with open(self.json_path, "w", encoding="utf-8") as handle:
                json.dump(result, handle, ensure_ascii=False, indent=2)
            self.png_path = self._write_plot(uniform_t, uniform_u, uniform_y, result)
            return result

    def _write_plot(self, t, u, y, result):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        frequency = np.asarray(result["frequency_hz"])
        gain = np.asarray(result["gain_db"])
        phase = np.asarray(result["phase_deg"])
        coherence = np.asarray(result["coherence"])
        fig, axes = plt.subplots(4, 1, figsize=(10, 13), constrained_layout=True)
        axes[0].plot(t, u, label="Command PWM", color="#2563eb")
        axes[0].plot(t, y, label="Detrended angle (deg)", color="#dc2626", alpha=0.8)
        axes[0].set_xlabel("Time (s)")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        positive = frequency > 0
        axes[1].semilogx(frequency[positive], gain[positive])
        axes[1].set_ylabel("Magnitude (dB)")
        axes[1].grid(True, which="both", alpha=0.3)
        axes[2].semilogx(frequency[positive], phase[positive])
        axes[2].set_ylabel("Phase (deg)")
        axes[2].grid(True, which="both", alpha=0.3)
        axes[3].semilogx(frequency[positive], coherence[positive])
        axes[3].axhline(self.config["coherence_threshold"], color="red", linestyle="--")
        axes[3].set_ylabel("Coherence")
        axes[3].set_xlabel("Frequency (Hz)")
        axes[3].set_ylim(0.0, 1.05)
        axes[3].grid(True, which="both", alpha=0.3)
        path = os.path.join(self.output_directory, "frequency_analysis.png")
        fig.savefig(path, dpi=150)
        plt.close(fig)
        return path
