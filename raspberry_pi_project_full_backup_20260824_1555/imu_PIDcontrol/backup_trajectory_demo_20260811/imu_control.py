import time
import math
import sys
import threading
import smbus

class Kalman1D:
    """
    一維 Kalman Filter，用於平滑角度量測。

    這裡使用簡化的一階狀態模型：
    x_k = x_{k-1} + w
    z_k = x_k + v

    適合放在 elbow_angle、shoulder_angle 這種已經估算好的角度後端，
    讓顯示、動作判斷與 PID 控制輸出更穩定。
    """

    def __init__(self, process_noise=0.03, measurement_noise=1.2, initial_error=1.0):
        self.q = process_noise
        self.r = measurement_noise
        self.p = initial_error
        self.x = 0.0
        self.initialized = False

    def update(self, measurement):
        if not self.initialized:
            self.x = measurement
            self.initialized = True
            return self.x

        # Prediction
        self.p = self.p + self.q

        # Correction
        k = self.p / (self.p + self.r)
        self.x = self.x + k * (measurement - self.x)
        self.p = (1.0 - k) * self.p

        return self.x

    def reset(self):
        self.p = 1.0
        self.x = 0.0
        self.initialized = False


class IMURehabSystem:
    """
    雙 IMU 上肢姿態估測與控制資料產生模組

    Channel 定義：
    - Channel 0: 上臂 IMU，用於上臂角度、肩關節抬舉角度與肩關節輔助判斷
    - Channel 1: 前臂 IMU，用於前臂角度與肘關節角度判斷
    - Channel 2: 保留，可偵測但目前不參與主要判斷

    重要改動：
    1. 校正時記錄各 IMU 的三軸加速度基準向量。
    2. 肩關節抬舉角度 = 校正時重力向量 與 目前重力向量 的夾角。
    3. 前平舉、側平舉只要讓上臂離開自然下垂姿勢，shoulder_angle 都會增加。
    4. 肩關節是否需要輔助使用 hysteresis，避免門檻附近一直跳動。
    """

    TCA_ADDR = 0x70
    MPU_ADDR = 0x68

    # MPU6050 register addresses
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C

    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F

    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47

    def __init__(
        self,
        bus_id=1,
        channels=(0, 1, 2),
        alpha_filter=0.96,
        ema_factor=0.3,
        vel_threshold=15.0,
        shoulder_vel_threshold=15.0,
        omega_sign=-1.0,
        shoulder_sign=1.0,
        target_mode="sine",
        target_joint="elbow",
        fixed_target_angle=60.0,
        sine_base=60.0,
        sine_amp=30.0,
        sine_freq=0.05,
        trajectory_min_angle=30.0,
        trajectory_max_angle=90.0,
        trajectory_period=5.0,
        step_hold_time=3.0,
        controller_mode="continuous_assist",
        deadband=1.5,
        deadband_on=2.0,
        deadband_off=1.0,
        assist_delay=0.10,
        assist_feedforward_gain=0.35,
        assist_feedforward_min_pwm=8.0,
        assist_feedforward_velocity_threshold=2.0,
        output_limit=50.0,
        motor_enabled=False,
        emergency_stop=False,
        kp=0.8,
        ki=0.02,
        kd=0.05,
        integral_limit=100.0,
        servo_min=0.0,
        servo_max=120.0,
        servo_neutral=60.0,
        control_sign=1.0,
        shoulder_angle_threshold=20.0,
        shoulder_release_threshold=12.0,
        shoulder_hold_velocity_threshold=6.0,
        shoulder_release_start_velocity=-8.0,
        shoulder_release_stop_velocity=-3.0,
        shoulder_release_base_pwm=8.0,
        shoulder_release_velocity_gain=0.7,
        shoulder_release_max_pwm=60.0,
        shoulder_release_fade_start_angle=15.0,
        shoulder_release_zero_angle=5.0,
        derivative_filter_tau=0.08,
        elbow_kalman_q=0.03,
        elbow_kalman_r=1.2,
        shoulder_kalman_q=0.03,
        shoulder_kalman_r=1.5,
        accel_norm_min=0.6,
        accel_norm_max=1.4,
        elbow_axis="roll",
        elbow_sign=1.0,
    ):
        self.bus_id = bus_id
        self.channels = channels

        self.alpha_filter = alpha_filter
        self.ema_factor = ema_factor
        self.vel_threshold = vel_threshold
        self.shoulder_vel_threshold = shoulder_vel_threshold

        self.omega_sign = omega_sign
        self.shoulder_sign = shoulder_sign

        self.target_mode = target_mode
        self.target_joint = target_joint
        self.fixed_target_angle = fixed_target_angle
        self.sine_base = sine_base
        self.sine_amp = sine_amp
        self.sine_freq = sine_freq
        self.trajectory_min_angle = trajectory_min_angle
        self.trajectory_max_angle = trajectory_max_angle
        self.trajectory_period = trajectory_period
        self.step_hold_time = step_hold_time

        self.controller_mode = controller_mode
        self.deadband = deadband
        self.deadband_on = float(deadband_on)
        self.deadband_off = float(deadband_off)
        self.assist_delay = assist_delay
        self.assist_feedforward_gain = max(float(assist_feedforward_gain), 0.0)
        self.assist_feedforward_min_pwm = max(float(assist_feedforward_min_pwm), 0.0)
        self.assist_feedforward_velocity_threshold = max(
            float(assist_feedforward_velocity_threshold), 0.0
        )
        self.output_limit = output_limit
        self.motor_enabled = motor_enabled
        self.emergency_stop = emergency_stop

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self.servo_min = servo_min
        self.servo_max = servo_max
        self.servo_neutral = servo_neutral
        self.control_sign = control_sign

        # 肩關節判斷門檻
        # threshold: 超過此角度，啟動肩關節抬舉 / 輔助狀態
        # release: 低於此角度，解除肩關節抬舉 / 輔助狀態
        self.shoulder_angle_threshold = shoulder_angle_threshold
        self.shoulder_release_threshold = shoulder_release_threshold

        # 肩關節速度判斷門檻：
        # shoulder_vel_threshold 用來判斷正在抬 / 正在放。
        # shoulder_hold_velocity_threshold 用來判斷是否接近維持不動。
        self.shoulder_hold_velocity_threshold = shoulder_hold_velocity_threshold
        self.shoulder_release_start_velocity = float(shoulder_release_start_velocity)
        self.shoulder_release_stop_velocity = float(shoulder_release_stop_velocity)
        self.shoulder_release_base_pwm = float(shoulder_release_base_pwm)
        self.shoulder_release_velocity_gain = float(shoulder_release_velocity_gain)
        self.shoulder_release_max_pwm = float(shoulder_release_max_pwm)
        self.shoulder_release_fade_start_angle = float(shoulder_release_fade_start_angle)
        self.shoulder_release_zero_angle = float(shoulder_release_zero_angle)
        self.derivative_filter_tau = max(float(derivative_filter_tau), 0.0)

        # Kalman filter 參數
        self.elbow_kalman_q = elbow_kalman_q
        self.elbow_kalman_r = elbow_kalman_r
        self.shoulder_kalman_q = shoulder_kalman_q
        self.shoulder_kalman_r = shoulder_kalman_r

        # 加速度向量有效範圍。動作太劇烈時，加速度不只包含重力，
        # 肩關節角度會比較容易被干擾，因此可以用此範圍做簡單保護。
        self.accel_norm_min = accel_norm_min
        self.accel_norm_max = accel_norm_max

        self.shoulder_assist_active = False
        self.shoulder_motion_state = "肩關節未抬舉"
        self.shoulder_release_command = False
        self.shoulder_release_pwm = 0.0
        self.shoulder_releasing = False
        self.shoulder_motor_enable = False

        # 肘角主要使用哪一個軸估算："roll" 或 "pitch"
        # 你原本使用 atan2(ay, az)，等價於 roll，所以預設 roll。
        self.elbow_axis = elbow_axis
        # Anatomical convention after validation: flexion is positive and
        # extension moves toward zero/negative.  Do not hide mounting errors
        # with abs(); the validation workflow determines this sign.
        self.elbow_sign = 1.0 if float(elbow_sign) >= 0.0 else -1.0
        self.shoulder_front_reference = None
        self.shoulder_side_reference = None
        self.patient_rom = None

        self.bus = None
        self.i2c_lock = threading.RLock()
        self.available_channels = []

        # base_angles[ch] = {"roll": ..., "pitch": ...}
        self.base_angles = {}

        # gyro_offsets[ch] = {"gx": ..., "gy": ..., "gz": ...}
        self.gyro_offsets = {}

        # current_angles[ch] = {"roll": ..., "pitch": ...}
        self.current_angles = {}

        # accel_baseline[ch] = normalized vector (ax, ay, az) at calibration posture
        self.accel_baseline = {}

        self.last_time = None
        self.data_start_time = None

        self.last_omega = 0.0
        self.last_alpha_acc = 0.0

        self.smooth_omega = 0.0
        self.smooth_alpha = 0.0
        self.smooth_jerk = 0.0

        self.smooth_shoulder_angle = 0.0
        self.last_shoulder_angle = 0.0
        self.shoulder_angle_velocity = 0.0
        self.shoulder_motion_state = "肩關節未抬舉"
        self.shoulder_release_command = False
        self.shoulder_motor_enable = False

        self.elbow_kalman = Kalman1D(
            process_noise=self.elbow_kalman_q,
            measurement_noise=self.elbow_kalman_r,
            initial_error=1.0,
        )
        self.shoulder_kalman = Kalman1D(
            process_noise=self.shoulder_kalman_q,
            measurement_noise=self.shoulder_kalman_r,
            initial_error=1.0,
        )

        self.integral_error = 0.0
        self.last_error = 0.0
        self.pid_initialized = False
        self.control_active = False
        self.last_measured_angle = 0.0
        self.filtered_measurement_velocity = 0.0
        self.shoulder_velocity_initialized = False
        self.error_active_time = 0.0
        self.derivative_error = 0.0
        self.error_for_control = 0.0
        self.raw_pid_output = 0.0
        self.limited_pid_output = 0.0
        self.anti_windup_active = False
        self.error_active_time = 0.0
        self.derivative_error = 0.0
        self.error_for_control = 0.0
        self.raw_pid_output = 0.0
        self.limited_pid_output = 0.0

    # ================== 基本工具 ==================

    def connect_bus(self):
        try:
            self.bus = smbus.SMBus(self.bus_id)
            return True
        except Exception as e:
            raise RuntimeError(f"I2C 總線初始化失敗: {e}")

    def select_channel(self, channel):
        self.bus.write_byte(self.TCA_ADDR, 1 << channel)

    def init_mpu(self):
        # Wake up MPU6050
        self.bus.write_byte_data(self.MPU_ADDR, self.PWR_MGMT_1, 0)
        time.sleep(0.02)

        # Basic low-noise configuration.
        # SMPLRT_DIV=4 -> internal 1kHz/(1+4)=200Hz sample rate under DLPF.
        # CONFIG=3 -> DLPF on, helps reduce high-frequency noise.
        # GYRO_CONFIG=0 -> ±250 deg/s, scale 131 LSB/(deg/s).
        # ACCEL_CONFIG=0 -> ±2g, scale 16384 LSB/g.
        self.bus.write_byte_data(self.MPU_ADDR, self.SMPLRT_DIV, 4)
        self.bus.write_byte_data(self.MPU_ADDR, self.CONFIG, 3)
        self.bus.write_byte_data(self.MPU_ADDR, self.GYRO_CONFIG, 0)
        self.bus.write_byte_data(self.MPU_ADDR, self.ACCEL_CONFIG, 0)

    def read_word_2c(self, addr):
        high = self.bus.read_byte_data(self.MPU_ADDR, addr)
        low = self.bus.read_byte_data(self.MPU_ADDR, addr + 1)
        val = (high << 8) + low

        if val >= 0x8000:
            return -((65535 - val) + 1)

        return val

    @staticmethod
    def _signed_word(high, low):
        value = (int(high) << 8) | int(low)
        return value - 65536 if value >= 0x8000 else value

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def normalize_vector(self, ax, ay, az):
        norm = math.sqrt(ax * ax + ay * ay + az * az)

        if norm < 1e-6:
            return 0.0, 0.0, 0.0

        return ax / norm, ay / norm, az / norm

    def accel_norm(self, ax, ay, az):
        return math.sqrt(ax * ax + ay * ay + az * az)

    def is_accel_vector_valid(self, ax, ay, az):
        norm = self.accel_norm(ax, ay, az)
        return self.accel_norm_min <= norm <= self.accel_norm_max

    def angle_between_vectors(self, v1, v2):
        dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
        dot = self.clamp(dot, -1.0, 1.0)
        return math.degrees(math.acos(dot))

    def get_target_angle(self, t):
        """
        產生復健訓練目標軌跡。

        本專題目前提供三種軌跡：
        - fixed：固定在 fixed_target_angle，用於單一關節初期閉迴路測試
        - sine：在 trajectory_min_angle 與 trajectory_max_angle 間做平滑正弦軌跡
        - step：每 step_hold_time 秒在最小與最大角度間切換，用於階躍響應測試
        """
        mode = str(self.target_mode).lower()
        if mode == "fixed":
            return float(self.fixed_target_angle)

        min_a = float(self.trajectory_min_angle)
        max_a = float(self.trajectory_max_angle)

        if max_a < min_a:
            min_a, max_a = max_a, min_a

        center = 0.5 * (min_a + max_a)
        amp = 0.5 * (max_a - min_a)
        period = max(float(self.trajectory_period), 0.1)

        if mode == "step":
            hold = max(float(self.step_hold_time), 0.1)
            index = int(t // hold)
            return max_a if index % 2 == 1 else min_a

        # 預設為 sine，避免 GUI 傳入其他字串時造成控制模式不明確。
        # Start at the minimum angle with zero velocity for safe motor arming.
        return center - amp * math.cos(2.0 * math.pi * t / period)

    def get_target_velocity(self, t):
        """Analytic trajectory velocity used by D control and feedforward."""
        mode = str(self.target_mode).lower()
        if mode != "sine":
            return 0.0
        min_a = float(self.trajectory_min_angle)
        max_a = float(self.trajectory_max_angle)
        if max_a < min_a:
            min_a, max_a = max_a, min_a
        amplitude = 0.5 * (max_a - min_a)
        period = max(float(self.trajectory_period), 0.1)
        omega = 2.0 * math.pi / period
        return amplitude * omega * math.sin(omega * float(t))

    def configure_control(self, reset_controller=True, **kwargs):
        """
        讓 Web GUI 可以即時更新軌跡、PID、deadband 與馬達旗標。
        """
        allowed = {
            "target_mode", "target_joint", "fixed_target_angle", "sine_base", "sine_amp", "sine_freq",
            "trajectory_min_angle", "trajectory_max_angle", "trajectory_period", "step_hold_time",
            "controller_mode", "deadband", "deadband_on", "deadband_off", "assist_delay", "output_limit",
            "assist_feedforward_gain", "assist_feedforward_min_pwm", "assist_feedforward_velocity_threshold",
            "kp", "ki", "kd", "integral_limit",
            "servo_min", "servo_max", "servo_neutral", "control_sign",
            "motor_enabled", "emergency_stop",
        }
        for key, value in kwargs.items():
            if key in allowed:
                setattr(self, key, value)

        if reset_controller:
            self.integral_error = 0.0
            self.last_error = 0.0
            self.pid_initialized = False
            self.control_active = False
            self.last_measured_angle = 0.0
            self.filtered_measurement_velocity = 0.0
            self.error_active_time = 0.0
            self.derivative_error = 0.0
            self.error_for_control = 0.0
            self.raw_pid_output = 0.0
            self.limited_pid_output = 0.0

    def apply_imu_validation_mapping(
        self,
        elbow_axis,
        elbow_sign,
        front_reference=None,
        side_reference=None,
    ):
        """Apply a user-confirmed mapping for the current initialization only."""
        axis = str(elbow_axis).lower()
        if axis not in ("roll", "pitch"):
            raise ValueError("elbow_axis must be roll or pitch")
        self.elbow_axis = axis
        self.elbow_sign = 1.0 if float(elbow_sign) >= 0.0 else -1.0

        def normalized_reference(value):
            if value is None or len(value) != 3:
                return None
            return self.normalize_vector(float(value[0]), float(value[1]), float(value[2]))

        self.shoulder_front_reference = normalized_reference(front_reference)
        self.shoulder_side_reference = normalized_reference(side_reference)
        self.elbow_kalman.reset()
        self.last_omega = 0.0
        self.smooth_omega = 0.0
        self.pid_initialized = False
        self.integral_error = 0.0

    def apply_rom_calibration(
        self,
        elbow_axis,
        elbow_sign,
        front_reference,
        side_reference,
        rom,
    ):
        """Apply session-relative joint mapping and patient-specific ROM."""
        self.apply_imu_validation_mapping(
            elbow_axis=elbow_axis,
            elbow_sign=elbow_sign,
            front_reference=front_reference,
            side_reference=side_reference,
        )
        self.patient_rom = dict(rom or {})
        return dict(self.patient_rom)

    # ================== MPU6050 讀取 ==================

    def get_raw_data(self):
        """
        回傳三軸加速度、三軸角速度，以及由加速度估出的 roll / pitch。

        注意：
        - roll = atan2(ay, az)，保留你原本使用的 Y-Z 平面角度。
        - pitch = atan2(-ax, sqrt(ay^2 + az^2))。
        - yaw 無法只靠 MPU6050 加速度穩定取得。
        """
        # MPU6050 exposes accel, temperature and gyro as one contiguous
        # 14-byte register block. One block transaction is both faster and
        # much less likely to wedge the I2C bus than twelve byte reads.
        block = self.bus.read_i2c_block_data(
            self.MPU_ADDR, self.ACCEL_XOUT_H, 14
        )
        if len(block) != 14:
            raise IOError(f"MPU6050 short read: expected 14 bytes, got {len(block)}")

        ax = self._signed_word(block[0], block[1]) / 16384.0
        ay = self._signed_word(block[2], block[3]) / 16384.0
        az = self._signed_word(block[4], block[5]) / 16384.0
        gx = self._signed_word(block[8], block[9]) / 131.0
        gy = self._signed_word(block[10], block[11]) / 131.0
        gz = self._signed_word(block[12], block[13]) / 131.0

        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        return {
            "ax": ax,
            "ay": ay,
            "az": az,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "roll": roll,
            "pitch": pitch,
        }

    def read_channel_raw(self, channel):
        """Atomically select one TCA9548A channel and read its MPU6050."""
        with self.i2c_lock:
            self.select_channel(channel)
            return self.get_raw_data()

    # ================== 初始化與校正 ==================

    def scan_and_init(self):
        if self.bus is None:
            self.connect_bus()

        self.available_channels = []

        for channel in self.channels:
            try:
                with self.i2c_lock:
                    self.select_channel(channel)
                    self.init_mpu()
                self.available_channels.append(channel)
            except Exception:
                pass

        if not self.available_channels:
            raise RuntimeError("沒有可用的 MPU6050，請檢查接線。")

        if not (0 in self.available_channels and 1 in self.available_channels):
            raise RuntimeError(
                f"本系統至少需要 Channel 0 上臂 IMU 與 Channel 1 前臂 IMU。"
                f"目前可用通道: {self.available_channels}"
            )

        return self.available_channels

    def calibrate(self, seconds=10, progress_callback=None):
        """
        校正姿勢：手臂自然下垂或你定義的零位姿勢。

        本版本會同時記錄：
        1. roll / pitch 的基準角度
        2. gx / gy / gz 的零飄偏移
        3. ax / ay / az 的平均重力向量，作為肩關節抬舉角度的基準向量
        """
        channel_data = {
            ch: {
                "sum_roll": 0.0,
                "sum_pitch": 0.0,
                "sum_gx": 0.0,
                "sum_gy": 0.0,
                "sum_gz": 0.0,
                "sum_ax": 0.0,
                "sum_ay": 0.0,
                "sum_az": 0.0,
            }
            for ch in self.available_channels
        }

        count = 0
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time

            if elapsed >= seconds:
                break

            if progress_callback is not None:
                remaining = max(0.0, seconds - elapsed)
                progress_callback(elapsed, remaining)

            for ch in self.available_channels:
                raw = self.read_channel_raw(ch)

                channel_data[ch]["sum_roll"] += raw["roll"]
                channel_data[ch]["sum_pitch"] += raw["pitch"]

                channel_data[ch]["sum_gx"] += raw["gx"]
                channel_data[ch]["sum_gy"] += raw["gy"]
                channel_data[ch]["sum_gz"] += raw["gz"]

                channel_data[ch]["sum_ax"] += raw["ax"]
                channel_data[ch]["sum_ay"] += raw["ay"]
                channel_data[ch]["sum_az"] += raw["az"]

            count += 1
            time.sleep(0.005)

        if count == 0:
            raise RuntimeError("校正失敗，沒有取得任何資料。")

        self.base_angles = {}
        self.gyro_offsets = {}
        self.current_angles = {}
        self.accel_baseline = {}

        for ch in self.available_channels:
            avg_roll = channel_data[ch]["sum_roll"] / count
            avg_pitch = channel_data[ch]["sum_pitch"] / count

            avg_gx = channel_data[ch]["sum_gx"] / count
            avg_gy = channel_data[ch]["sum_gy"] / count
            avg_gz = channel_data[ch]["sum_gz"] / count

            avg_ax = channel_data[ch]["sum_ax"] / count
            avg_ay = channel_data[ch]["sum_ay"] / count
            avg_az = channel_data[ch]["sum_az"] / count

            self.base_angles[ch] = {
                "roll": avg_roll,
                "pitch": avg_pitch,
            }

            self.gyro_offsets[ch] = {
                "gx": avg_gx,
                "gy": avg_gy,
                "gz": avg_gz,
            }

            self.current_angles[ch] = {
                "roll": avg_roll,
                "pitch": avg_pitch,
            }

            self.accel_baseline[ch] = self.normalize_vector(avg_ax, avg_ay, avg_az)

        self.reset_runtime_states()

        return {
            "base_angles": self.base_angles,
            "gyro_offsets": self.gyro_offsets,
            "accel_baseline": self.accel_baseline,
            "sample_count": count,
        }

    def reset_runtime_states(self):
        self.last_time = time.time()
        self.data_start_time = self.last_time

        self.last_omega = 0.0
        self.last_alpha_acc = 0.0

        self.smooth_omega = 0.0
        self.smooth_alpha = 0.0
        self.smooth_jerk = 0.0

        self.smooth_shoulder_angle = 0.0
        self.last_shoulder_angle = 0.0
        self.shoulder_angle_velocity = 0.0
        self.shoulder_assist_active = False
        self.shoulder_motion_state = "肩關節未抬舉"
        self.shoulder_release_command = False
        self.shoulder_release_pwm = 0.0
        self.shoulder_releasing = False
        self.shoulder_motor_enable = False

        self.elbow_kalman.reset()
        self.shoulder_kalman.reset()

        self.integral_error = 0.0
        self.last_error = 0.0
        self.pid_initialized = False
        self.control_active = False
        self.last_measured_angle = 0.0
        self.filtered_measurement_velocity = 0.0
        self.shoulder_velocity_initialized = False
        self.error_active_time = 0.0
        self.derivative_error = 0.0
        self.error_for_control = 0.0
        self.raw_pid_output = 0.0
        self.limited_pid_output = 0.0
        self.error_active_time = 0.0
        self.derivative_error = 0.0
        self.error_for_control = 0.0
        self.raw_pid_output = 0.0
        self.limited_pid_output = 0.0
        self.anti_windup_active = False

    # ================== 即時讀取與特徵計算 ==================

    def update_shoulder_release(self, shoulder_angle, shoulder_velocity):
        """Return a positive payout magnitude with velocity hysteresis and angle fade."""
        angle = max(0.0, float(shoulder_angle))
        velocity = float(shoulder_velocity)

        if not self.shoulder_releasing:
            if (
                velocity <= self.shoulder_release_start_velocity
                and angle > self.shoulder_release_zero_angle
            ):
                self.shoulder_releasing = True
        elif (
            velocity >= self.shoulder_release_stop_velocity
            or angle <= self.shoulder_release_zero_angle
        ):
            self.shoulder_releasing = False

        if not self.shoulder_releasing:
            self.shoulder_release_pwm = 0.0
            return 0.0

        fade_span = max(
            self.shoulder_release_fade_start_angle - self.shoulder_release_zero_angle,
            1e-6,
        )
        angle_scale = self.clamp(
            (angle - self.shoulder_release_zero_angle) / fade_span,
            0.0,
            1.0,
        )
        down_speed = max(0.0, -velocity)
        payout = (
            self.shoulder_release_base_pwm
            + self.shoulder_release_velocity_gain * down_speed
        ) * angle_scale
        self.shoulder_release_pwm = self.clamp(
            payout,
            0.0,
            self.shoulder_release_max_pwm,
        )
        return self.shoulder_release_pwm

    def compute_pid_output(self, target_angle, measured_angle, dt, target_velocity=0.0):
        """PID with Schmitt error gating, filtered D-on-measurement and anti-windup."""
        dt = max(float(dt), 1e-4)
        error = float(target_angle) - float(measured_angle)
        abs_error = abs(error)
        on_threshold = max(self.deadband_on, self.deadband_off)
        off_threshold = min(self.deadband_on, self.deadband_off)

        if self.control_active:
            if abs_error <= off_threshold:
                self.control_active = False
        elif abs_error >= on_threshold:
            self.control_active = True

        if self.control_active:
            self.error_for_control = error
            self.error_active_time += dt
        else:
            self.error_for_control = 0.0
            self.error_active_time = 0.0

        if not self.pid_initialized:
            raw_measurement_velocity = 0.0
            self.filtered_measurement_velocity = 0.0
            self.pid_initialized = True
        else:
            raw_measurement_velocity = (
                float(measured_angle) - self.last_measured_angle
            ) / dt
            if self.derivative_filter_tau <= 0.0:
                self.filtered_measurement_velocity = raw_measurement_velocity
            else:
                alpha = dt / (self.derivative_filter_tau + dt)
                self.filtered_measurement_velocity += alpha * (
                    raw_measurement_velocity - self.filtered_measurement_velocity
                )

        self.last_measured_angle = float(measured_angle)
        self.last_error = error
        derivative_error = float(target_velocity) - self.filtered_measurement_velocity
        self.derivative_error = derivative_error

        mode = str(self.controller_mode).lower()
        continuous_modes = ("continuous_assist", "continuous", "feedforward_assist")
        use_p = mode in ("p", "pi", "pid", "assist", "assist_as_needed", "aan") + continuous_modes
        use_i = mode in ("pi", "pid") or (
            mode in ("assist", "assist_as_needed", "aan") + continuous_modes
            and self.error_active_time >= self.assist_delay
        )
        use_d = mode in ("pid", "assist", "assist_as_needed", "aan") + continuous_modes
        if not self.control_active:
            use_p = False
            use_i = False
            use_d = False

        p_term = self.kp * self.error_for_control if use_p else 0.0
        d_term = self.kd * derivative_error if use_d else 0.0
        self.anti_windup_active = False

        if use_i:
            candidate_integral = self.clamp(
                self.integral_error + self.error_for_control * dt,
                -self.integral_limit,
                self.integral_limit,
            )
            candidate_raw = p_term + self.ki * candidate_integral + d_term
            drives_high_saturation = (
                candidate_raw > self.output_limit and self.error_for_control > 0.0
            )
            drives_low_saturation = (
                candidate_raw < -self.output_limit and self.error_for_control < 0.0
            )
            if drives_high_saturation or drives_low_saturation:
                self.anti_windup_active = True
            else:
                self.integral_error = candidate_integral
        else:
            self.integral_error *= 0.98
            if abs(self.integral_error) < 1e-3:
                self.integral_error = 0.0

        i_term = self.ki * self.integral_error if use_i else 0.0
        raw_pid_output = p_term + i_term + d_term
        pid_output = self.clamp(raw_pid_output, -self.output_limit, self.output_limit)

        self.raw_pid_output = raw_pid_output
        self.limited_pid_output = pid_output
        return error, pid_output

    def compute_assist_feedforward(self, target_velocity):
        mode = str(self.controller_mode).lower()
        if mode not in ("continuous_assist", "continuous", "feedforward_assist"):
            return 0.0
        velocity = float(target_velocity)
        if abs(velocity) < self.assist_feedforward_velocity_threshold:
            return 0.0
        magnitude = max(
            self.assist_feedforward_min_pwm,
            self.assist_feedforward_gain * abs(velocity),
        )
        return self.clamp(
            math.copysign(magnitude, velocity),
            -self.output_limit,
            self.output_limit,
        )

    def read_frame(self):
        if self.last_time is None or self.data_start_time is None:
            self.reset_runtime_states()

        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0:
            dt = 0.001

        self.last_time = current_time
        t = current_time - self.data_start_time

        raw_data = {}
        gyro_corr_data = {}
        accel_unit_data = {}
        accel_valid_data = {}

        # ---------- 讀取所有 IMU ----------
        for ch in self.available_channels:
            raw = self.read_channel_raw(ch)
            raw_data[ch] = raw

            gx_corr = raw["gx"] - self.gyro_offsets[ch]["gx"]
            gy_corr = raw["gy"] - self.gyro_offsets[ch]["gy"]
            gz_corr = raw["gz"] - self.gyro_offsets[ch]["gz"]

            gyro_corr_data[ch] = {
                "gx": gx_corr,
                "gy": gy_corr,
                "gz": gz_corr,
            }

            accel_unit_data[ch] = self.normalize_vector(raw["ax"], raw["ay"], raw["az"])
            accel_valid_data[ch] = self.is_accel_vector_valid(raw["ax"], raw["ay"], raw["az"])

            # roll 使用 gx 做互補濾波
            self.current_angles[ch]["roll"] = (
                self.alpha_filter * (self.current_angles[ch]["roll"] + gx_corr * dt)
                + (1.0 - self.alpha_filter) * raw["roll"]
            )

            # pitch 使用 gy 做互補濾波
            self.current_angles[ch]["pitch"] = (
                self.alpha_filter * (self.current_angles[ch]["pitch"] + gy_corr * dt)
                + (1.0 - self.alpha_filter) * raw["pitch"]
            )

        # ---------- 相對角度 ----------
        rel_angles = {}
        for ch in self.available_channels:
            rel_angles[ch] = {
                "roll": self.current_angles[ch]["roll"] - self.base_angles[ch]["roll"],
                "pitch": self.current_angles[ch]["pitch"] - self.base_angles[ch]["pitch"],
            }

        upper_roll = rel_angles[0]["roll"]
        upper_pitch = rel_angles[0]["pitch"]
        forearm_roll = rel_angles[1]["roll"]
        forearm_pitch = rel_angles[1]["pitch"]

        elbow_roll_signed = forearm_roll - upper_roll
        elbow_pitch_signed = forearm_pitch - upper_pitch

        if self.elbow_axis == "pitch":
            upper_angle = upper_pitch
            forearm_angle = forearm_pitch
            signed_elbow_angle = self.elbow_sign * elbow_pitch_signed
            raw_omega = self.elbow_sign * (
                gyro_corr_data[1]["gy"] - gyro_corr_data[0]["gy"]
            )
        else:
            upper_angle = upper_roll
            forearm_angle = forearm_roll
            signed_elbow_angle = self.elbow_sign * elbow_roll_signed
            raw_omega = self.elbow_sign * (
                gyro_corr_data[1]["gx"] - gyro_corr_data[0]["gx"]
            )

        elbow_angle_raw = signed_elbow_angle
        elbow_angle = self.elbow_kalman.update(elbow_angle_raw)

        # ---------- 肘關節角速度 / 角加速度 / Jerk ----------
        self.smooth_omega = (
            self.ema_factor * raw_omega
            + (1.0 - self.ema_factor) * self.smooth_omega
        )

        raw_alpha_acc = (self.smooth_omega - self.last_omega) / dt
        self.smooth_alpha = (
            self.ema_factor * raw_alpha_acc
            + (1.0 - self.ema_factor) * self.smooth_alpha
        )

        raw_jerk = (self.smooth_alpha - self.last_alpha_acc) / dt
        self.smooth_jerk = (
            self.ema_factor * raw_jerk
            + (1.0 - self.ema_factor) * self.smooth_jerk
        )

        self.last_omega = self.smooth_omega
        self.last_alpha_acc = self.smooth_alpha

        # ---------- 肘關節動作狀態 ----------
        if self.smooth_omega > self.vel_threshold:
            motion_state = "肘屈曲"
        elif self.smooth_omega < -self.vel_threshold:
            motion_state = "肘伸展"
        else:
            motion_state = "靜止"

        # ---------- 肩關節抬舉角度：重力向量偏移 ----------
        upper_base_vector = self.accel_baseline[0]
        upper_current_vector = accel_unit_data[0]

        shoulder_angle_raw = self.angle_between_vectors(
            upper_base_vector,
            upper_current_vector,
        )

        # 若手臂快速晃動，accelerometer 會混入動態加速度；
        # 若加速度長度太偏離 1g，先保留上一筆肩角，避免誤判。
        shoulder_accel_valid = accel_valid_data.get(0, True)
        if shoulder_accel_valid:
            self.smooth_shoulder_angle = self.shoulder_kalman.update(
                shoulder_angle_raw
            )
        else:
            # 不更新 Kalman，但保留上一個估測值。
            self.smooth_shoulder_angle = self.shoulder_kalman.x

        if not self.shoulder_velocity_initialized:
            self.shoulder_angle_velocity = 0.0
            self.shoulder_velocity_initialized = True
        else:
            self.shoulder_angle_velocity = (
                self.smooth_shoulder_angle - self.last_shoulder_angle
            ) / dt

        self.last_shoulder_angle = self.smooth_shoulder_angle

        # 判斷前平舉 / 側平舉：先用 X / Z 軸主導，實測後可對應到前平舉或側平舉
        dx = upper_current_vector[0] - upper_base_vector[0]
        dy = upper_current_vector[1] - upper_base_vector[1]
        dz = upper_current_vector[2] - upper_base_vector[2]

        shoulder_delta = self.normalize_vector(dx, dy, dz)
        if self.shoulder_front_reference is not None and self.shoulder_side_reference is not None:
            front_similarity = sum(
                a * b for a, b in zip(shoulder_delta, self.shoulder_front_reference)
            )
            side_similarity = sum(
                a * b for a, b in zip(shoulder_delta, self.shoulder_side_reference)
            )
            shoulder_plane = "前平舉" if front_similarity >= side_similarity else "側平舉"
        elif abs(dx) >= abs(dz):
            shoulder_plane = "X軸主導抬舉（尚未驗證）"
        else:
            shoulder_plane = "Z軸主導抬舉（尚未驗證）"

        # Hysteresis：避免在門檻附近一直跳動
        # 注意：shoulder_assist_active 表示「肩關節仍處於抬起區間」，
        # 不代表馬達一定要繼續出力。
        # 若偵測到正在下放，會另外輸出 shoulder_release_command=True，
        # 之後接馬達時可用來鬆掉或降低輔助。
        if not self.shoulder_assist_active:
            if self.smooth_shoulder_angle >= self.shoulder_angle_threshold:
                self.shoulder_assist_active = True
        else:
            if self.smooth_shoulder_angle <= self.shoulder_release_threshold:
                self.shoulder_assist_active = False

        # ---------- 肩關節動作方向判斷 ----------
        # velocity > 0：肩角正在增加，代表上臂正在離開自然下垂位置
        # velocity < 0：肩角正在減少，代表上臂正在回到自然下垂位置
        shoulder_release_pwm = self.update_shoulder_release(
            self.smooth_shoulder_angle,
            self.shoulder_angle_velocity,
        )

        if self.shoulder_releasing:
            self.shoulder_motion_state = "肩關節下放中"
        elif self.shoulder_angle_velocity > self.shoulder_vel_threshold:
            self.shoulder_motion_state = "肩關節抬舉中"
        else:
            if self.shoulder_assist_active:
                self.shoulder_motion_state = "肩關節維持抬舉"
            else:
                self.shoulder_motion_state = "肩關節未抬舉"

        # 接馬達時的控制旗標：
        # 1. shoulder_release_command=True：病人正在把手放下，馬達應降低輔助或回中立。
        # 2. shoulder_motor_enable=True：只有在抬舉或維持抬舉時才允許肩關節輔助。
        self.shoulder_release_command = shoulder_release_pwm > 0.0

        self.shoulder_motor_enable = (
            self.shoulder_assist_active
            and not self.shoulder_release_command
        )

        if self.shoulder_motion_state == "肩關節抬舉中":
            motion_state += f"＋肩關節抬舉中({shoulder_plane})"
        elif self.shoulder_motion_state == "肩關節下放中":
            motion_state += "＋肩關節下放中"
        elif self.shoulder_motion_state == "肩關節維持抬舉":
            motion_state += f"＋肩關節維持抬舉({shoulder_plane})"

        # ---------- 目標關節選擇、目標軌跡與按需輔助控制輸出 ----------
        target_angle = self.get_target_angle(t)
        target_velocity = self.get_target_velocity(t)
        target_joint = str(self.target_joint).lower()

        if target_joint in ("shoulder", "shoulder_joint", "肩關節"):
            controlled_joint = "shoulder"
            measured_angle = self.smooth_shoulder_angle
            measured_angle_raw = shoulder_angle_raw
        else:
            controlled_joint = "elbow"
            measured_angle = elbow_angle
            measured_angle_raw = elbow_angle_raw

        if (
            controlled_joint == "shoulder"
            and str(self.controller_mode).lower()
            in ("continuous_assist", "continuous", "feedforward_assist")
        ):
            # Continuous trajectory following must be able to wind from the
            # natural-down position. Actual downward motion still activates
            # shoulder_release_command and the velocity-scaled payout path.
            self.shoulder_motor_enable = not self.shoulder_release_command

        error, feedback_pid_output = self.compute_pid_output(
            target_angle,
            measured_angle,
            dt,
            target_velocity=target_velocity,
        )
        feedforward_output = self.compute_assist_feedforward(target_velocity)
        pid_output = self.clamp(
            feedback_pid_output + feedforward_output,
            -self.output_limit,
            self.output_limit,
        )

        if self.emergency_stop or not self.motor_enabled:
            servo_cmd = self.servo_neutral
            motor_cmd = 0.0
        else:
            servo_cmd = self.servo_neutral + self.control_sign * pid_output
            servo_cmd = self.clamp(servo_cmd, self.servo_min, self.servo_max)
            motor_cmd = pid_output

        return {
            "time": t,

            # 為了相容原本 Web GUI，upper_angle / forearm_angle 保留為主要肘角軸
            "upper_angle": upper_angle,
            "forearm_angle": forearm_angle,
            "signed_elbow_angle": signed_elbow_angle,
            "elbow_roll_signed": elbow_roll_signed,
            "elbow_pitch_signed": elbow_pitch_signed,
            "elbow_axis": self.elbow_axis,
            "elbow_sign": self.elbow_sign,
            "elbow_angle_raw": elbow_angle_raw,
            "elbow_angle": elbow_angle,

            # 新增：多軸角度，方便你之後 debug 前平舉 / 側平舉
            "upper_roll": upper_roll,
            "upper_pitch": upper_pitch,
            "forearm_roll": forearm_roll,
            "forearm_pitch": forearm_pitch,

            "omega": self.smooth_omega,
            "alpha": self.smooth_alpha,
            "jerk": self.smooth_jerk,

            # 新增：肩關節角度與輔助判斷
            "shoulder_angle": self.smooth_shoulder_angle,
            "shoulder_angle_raw": shoulder_angle_raw,
            "shoulder_angle_filtered": self.smooth_shoulder_angle,
            "shoulder_accel_valid": shoulder_accel_valid,
            "shoulder_angle_velocity": self.shoulder_angle_velocity,
            "shoulder_plane": shoulder_plane,
            "shoulder_assist": self.shoulder_assist_active,
            "shoulder_motion_state": self.shoulder_motion_state,
            "shoulder_release_command": self.shoulder_release_command,
            "shoulder_release_pwm": self.shoulder_release_pwm,
            "shoulder_motor_enable": self.shoulder_motor_enable,
            "shoulder_dx": dx,
            "shoulder_dy": dy,
            "shoulder_dz": dz,

            "motion_state": motion_state,
            "target_joint": controlled_joint,
            "measured_angle": measured_angle,
            "measured_angle_raw": measured_angle_raw,
            "target_angle": target_angle,
            "target_velocity": target_velocity,
            "target_mode": self.target_mode,
            "trajectory_min_angle": self.trajectory_min_angle,
            "trajectory_max_angle": self.trajectory_max_angle,
            "trajectory_period": self.trajectory_period,
            "controller_mode": self.controller_mode,
            "deadband": self.deadband,
            "deadband_on": self.deadband_on,
            "deadband_off": self.deadband_off,
            "control_active": self.control_active,
            "error": error,
            "error_for_control": self.error_for_control,
            "error_active_time": self.error_active_time,
            "integral_error": self.integral_error,
            "derivative_error": self.derivative_error,
            "filtered_measurement_velocity": self.filtered_measurement_velocity,
            "anti_windup_active": self.anti_windup_active,
            "raw_pid_output": self.raw_pid_output,
            "feedback_pid_output": feedback_pid_output,
            "feedforward_output": feedforward_output,
            "pid_output": pid_output,
            "motor_cmd": motor_cmd,
            "servo_cmd": servo_cmd,
            "motor_enabled": self.motor_enabled,
            "emergency_stop": self.emergency_stop,
        }


if __name__ == "__main__":
    system = IMURehabSystem()

    print("初始化 IMU...")
    channels = system.scan_and_init()
    print(f"可用通道: {channels}")

    print("請將手臂自然下垂並保持靜止，開始 10 秒校正。")

    def progress(elapsed, remaining):
        print(f"校正中，剩餘 {remaining:.1f} 秒", end="\r")

    result = system.calibrate(seconds=10, progress_callback=progress)

    print("\n校正完成")
    print(result)

    print("開始即時輸出，按 Ctrl+C 結束。")

    try:
        while True:
            frame = system.read_frame()
            print(
                f"t={frame['time']:.2f}s | "
                f"肘角={frame['elbow_angle']:.1f}° | "
                f"肩角={frame['shoulder_angle']:.1f}° | "
                f"肩狀態={frame['shoulder_motion_state']} | "
                f"馬達允許={frame['shoulder_motor_enable']} | "
                f"放下命令={frame['shoulder_release_command']} | "
                f"目標={frame['target_angle']:.1f}° | "
                f"Servo={frame['servo_cmd']:.1f} | "
                f"狀態={frame['motion_state']}"
            )
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n結束測試。")
        sys.exit(0)
