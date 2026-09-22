# esp32_motor_bridge.py
# Raspberry Pi <-> ESP32 USB Serial bridge
# Pi sends: PWM,pwm_biceps,pwm_triceps,pwm_deltoid
# ESP32 returns: FB,count1,count2,count3,vel1,vel2,vel3,pwm1,pwm2,pwm3

import threading
import time
from dataclasses import dataclass

try:
    import serial
except ImportError as exc:
    serial = None


@dataclass
class MotorFeedback:
    count1: int = 0
    count2: int = 0
    count3: int = 0
    vel1: float = 0.0
    vel2: float = 0.0
    vel3: float = 0.0
    pwm1: int = 0
    pwm2: int = 0
    pwm3: int = 0
    last_update_time: float = 0.0


class ESP32MotorBridge:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.05, pwm_limit=255):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.pwm_limit = int(pwm_limit)
        self.ser = None
        self.reader_thread = None
        self.reader_running = False
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        # STOP -> ARM is one normal-start transaction. A concurrent browser
        # STOP waits until ARM is confirmed, then returns the node to IDLE.
        # ESTOP intentionally remains outside this lock so it can pre-empt ARM.
        self.command_lock = threading.RLock()
        self._rx_buffer = bytearray()
        self.feedback = MotorFeedback()
        self.connected = False
        self.last_line = ""          # last line received from ESP32
        self.last_error = ""
        self.last_sent_line = ""     # last command sent to ESP32
        self.sent_count = 0
        self.last_response = ""
        self.safety_state = "UNKNOWN"

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial 尚未安裝，請先執行：sudo apt install python3-serial")

        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        # CP2102 adapters can otherwise leave the ESP32 reset/boot lines asserted.
        self.ser.dtr = False
        self.ser.rts = False
        time.sleep(2.0)  # ESP32 often resets when serial opens
        self.connected = True
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.ping()
        return True

    def close(self):
        self.reader_running = False
        try:
            self.stop()
        except Exception:
            pass
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def clamp_pwm(self, value):
        value = int(round(value))
        if value > self.pwm_limit:
            return self.pwm_limit
        if value < -self.pwm_limit:
            return -self.pwm_limit
        return value

    def send_line(self, line):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("ESP32 serial 尚未連線")

        clean_line = line.strip()
        # Control, jog and emergency routes can run on different threads.
        # Serialize complete lines so two writes can never interleave.
        with self.write_lock:
            self.ser.write((clean_line + "\n").encode("utf-8"))
            self.last_sent_line = clean_line
            self.sent_count += 1

    def set_pwm(self, pwm_biceps, pwm_triceps, pwm_deltoid):
        p1 = self.clamp_pwm(pwm_biceps)
        p2 = self.clamp_pwm(pwm_triceps)
        p3 = self.clamp_pwm(pwm_deltoid)
        self.send_line(f"PWM,{p1},{p2},{p3}")

    def stop(self):
        with self.command_lock:
            if self.ser is not None and self.ser.is_open:
                self.send_line("STOP")

    def arm(self, wait_timeout=1.0):
        with self.command_lock:
            self.last_response = ""
            self.last_error = ""
            self.safety_state = "ARMING"
            self.send_line("ARM")
            deadline = time.monotonic() + float(wait_timeout)
            while time.monotonic() < deadline:
                if self.safety_state == "ARMED" or self.last_response == "OK,ARM,ARMED":
                    return True
                if self.last_error.startswith("ERR,"):
                    raise RuntimeError(f"ESP32 refused ARM: {self.last_error}")
                time.sleep(0.01)
            raise RuntimeError("ESP32 ARM confirmation timed out")

    def prepare_active(self, wait_timeout=1.0):
        """Atomically return ESP32 to IDLE and ARM it for a new output session."""
        with self.command_lock:
            self.stop()
            return self.arm(wait_timeout=wait_timeout)

    def emergency_stop(self, wait_timeout=1.0):
        self.last_response = ""
        self.last_error = ""
        self.safety_state = "ESTOPPING"
        self.send_line("ESTOP")
        deadline = time.monotonic() + float(wait_timeout)
        while time.monotonic() < deadline:
            if self.safety_state in ("ESTOP", "EMERGENCY_STOP") or self.last_response.startswith("OK,ESTOP"):
                return True
            if self.last_error.startswith("ERR,"):
                raise RuntimeError(f"ESP32 refused ESTOP: {self.last_error}")
            time.sleep(0.01)
        raise RuntimeError("ESP32 ESTOP confirmation timed out")

    def clear_fault(self, wait_timeout=1.0):
        self.last_response = ""
        self.last_error = ""
        self.safety_state = "CLEARING"
        self.send_line("CLEAR")
        deadline = time.monotonic() + float(wait_timeout)
        while time.monotonic() < deadline:
            if self.safety_state == "IDLE" or self.last_response.startswith("OK,CLEAR"):
                return True
            if self.last_error.startswith("ERR,"):
                raise RuntimeError(f"ESP32 refused CLEAR: {self.last_error}")
            time.sleep(0.01)
        raise RuntimeError("ESP32 CLEAR confirmation timed out")

    def zero_encoders(self):
        self.send_line("ZERO")

    def ping(self):
        self.send_line("PING")

    def get_feedback(self):
        with self.lock:
            return MotorFeedback(**self.feedback.__dict__)

    def _reader_loop(self):
        while self.reader_running:
            try:
                waiting = getattr(self.ser, "in_waiting", 0)
                chunk = self.ser.read(max(1, waiting))
                if not chunk:
                    continue
                self._rx_buffer.extend(chunk)
                if len(self._rx_buffer) > 4096 and b"\n" not in self._rx_buffer:
                    self._rx_buffer.clear()
                    self.last_error = "ESP32 serial receive line exceeded 4096 bytes"
                    continue
                while b"\n" in self._rx_buffer:
                    raw_line, _, remainder = self._rx_buffer.partition(b"\n")
                    self._rx_buffer = bytearray(remainder)
                    line = raw_line.decode("utf-8", errors="ignore").strip()
                    if line:
                        self._handle_line(line)
            except Exception as exc:
                self.last_error = str(exc)
                time.sleep(0.05)

    def _handle_line(self, line):
        self.last_line = line
        if line.startswith("FB,"):
            self._parse_feedback(line)
        elif line.startswith("STATE,"):
            parts = line.split(",")
            if len(parts) >= 2:
                self.safety_state = parts[1]
        elif line.startswith("OK,"):
            self.last_response = line
            if line == "OK,ARM,ARMED":
                self.safety_state = "ARMED"
            elif line.startswith("OK,ESTOP"):
                self.safety_state = "ESTOP"
            elif line.startswith("OK,STOP,") or line.startswith("OK,CLEAR"):
                self.safety_state = "IDLE"
        elif line.startswith("ERR,"):
            self.last_error = line

    def _parse_feedback(self, line):
        parts = line.split(",")
        if len(parts) != 10:
            return
        try:
            fb = MotorFeedback(
                count1=int(parts[1]),
                count2=int(parts[2]),
                count3=int(parts[3]),
                vel1=float(parts[4]),
                vel2=float(parts[5]),
                vel3=float(parts[6]),
                pwm1=int(parts[7]),
                pwm2=int(parts[8]),
                pwm3=int(parts[9]),
                last_update_time=time.time(),
            )
            with self.lock:
                self.feedback = fb
        except ValueError:
            return


if __name__ == "__main__":
    bridge = ESP32MotorBridge(port="/dev/ttyUSB0")
    try:
        bridge.connect()
        print("ESP32 connected")
        bridge.zero_encoders()
        bridge.set_pwm(60, 0, 0)
        time.sleep(1)
        bridge.set_pwm(-40, 0, 0)
        time.sleep(1)
        bridge.stop()
        for _ in range(20):
            print(bridge.get_feedback())
            time.sleep(0.1)
    finally:
        bridge.stop()
        bridge.close()
