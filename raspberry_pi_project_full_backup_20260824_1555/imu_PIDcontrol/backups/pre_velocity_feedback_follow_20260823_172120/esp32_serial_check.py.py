# esp32_serial_check.py
# 用 SSH 在 Raspberry Pi 上測 ESP32 USB Serial
# 測試：PING、ZERO、PWM 指令、FB 回傳

import serial
import sys
import time


def main():
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    else:
        port = "/dev/ttyUSB0"

    baudrate = 115200

    print(f"Opening {port} at {baudrate} baud...")

    ser = serial.Serial(port, baudrate, timeout=0.1)
    time.sleep(2.0)  # ESP32 開啟 serial 後通常會 reset，等它重開

    def send(cmd):
        print(f">>> {cmd}")
        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()

    def read_for(seconds=2.0):
        start = time.time()
        while time.time() - start < seconds:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print("<<<", line)

    try:
        print("Reading boot messages / feedback...")
        read_for(2.0)

        send("PING")
        read_for(1.0)

        send("ZERO")
        read_for(1.0)

        print("\nTest Motor 1 biceps +PWM")
        send("PWM,60,0,0")
        read_for(2.0)

        print("\nTest Motor 1 biceps -PWM")
        send("PWM,-60,0,0")
        read_for(2.0)

        print("\nTest Motor 2 triceps +PWM")
        send("PWM,0,60,0")
        read_for(2.0)

        print("\nTest Motor 2 triceps -PWM")
        send("PWM,0,-60,0")
        read_for(2.0)

        print("\nTest Motor 3 deltoid +PWM")
        send("PWM,0,0,60")
        read_for(2.0)

        print("\nTest Motor 3 deltoid -PWM")
        send("PWM,0,0,-60")
        read_for(2.0)

        print("\nStop all motors")
        send("STOP")
        read_for(1.0)

    finally:
        try:
            send("STOP")
        except Exception:
            pass
        ser.close()
        print("Closed.")


if __name__ == "__main__":
    main()