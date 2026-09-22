import unittest
from unittest import mock

from imu_control import IMURehabSystem


class FakeBus:
    def __init__(self):
        self.channel_writes = []
        self.block_reads = []

    def write_byte(self, address, value):
        self.channel_writes.append((address, value))

    def read_i2c_block_data(self, address, register, length):
        self.block_reads.append((address, register, length))
        return [
            0x40, 0x00,  # ax = +1 g
            0x00, 0x00,  # ay = 0 g
            0xC0, 0x00,  # az = -1 g
            0x00, 0x00,  # temperature (unused)
            0x00, 0x83,  # gx = +1 deg/s
            0xFE, 0xFA,  # gy = -2 deg/s
            0x00, 0x00,  # gz = 0 deg/s
        ]

    def write_byte_data(self, address, register, value):
        pass

    def close(self):
        self.closed = True


class FlakyBus(FakeBus):
    def __init__(self, failures=1):
        super().__init__()
        self.failures = failures

    def read_i2c_block_data(self, address, register, length):
        if self.failures > 0:
            self.failures -= 1
            raise OSError(121, "Remote I/O error")
        return super().read_i2c_block_data(address, register, length)


class IMUBlockReadTests(unittest.TestCase):
    def test_mux_selection_and_sensor_read_are_one_block_transaction(self):
        system = IMURehabSystem(channels=(0, 1))
        system.bus = FakeBus()

        raw = system.read_channel_raw(1)

        self.assertEqual(system.bus.channel_writes, [(system.TCA_ADDR, 0b10)])
        self.assertEqual(
            system.bus.block_reads,
            [(system.MPU_ADDR, system.ACCEL_XOUT_H, 14)],
        )
        self.assertAlmostEqual(raw["ax"], 1.0)
        self.assertAlmostEqual(raw["az"], -1.0)
        self.assertAlmostEqual(raw["gx"], 1.0)
        self.assertAlmostEqual(raw["gy"], -2.0)

    def test_reconnect_preserves_calibration_and_reopens_both_imus(self):
        system = IMURehabSystem(channels=(0, 1))
        old_bus = FakeBus()
        new_bus = FakeBus()
        system.bus = old_bus
        system.base_angles = {0: {"roll": 1.0, "pitch": 2.0}, 1: {"roll": 3.0, "pitch": 4.0}}
        system.gyro_offsets = {0: {"gx": 0.1, "gy": 0.2, "gz": 0.3}, 1: {"gx": 0.4, "gy": 0.5, "gz": 0.6}}
        system.accel_baseline = {0: (0.0, 0.0, 1.0), 1: (0.0, 0.0, 1.0)}
        saved_baseline = dict(system.base_angles)

        with mock.patch("imu_control.smbus.SMBus", return_value=new_bus):
            channels = system.reconnect_imus(retries=1, retry_delay=0.0)

        self.assertTrue(old_bus.closed)
        self.assertEqual(channels, [0, 1])
        self.assertEqual(system.base_angles, saved_baseline)
        self.assertEqual(system.reconnect_count, 1)

    def test_brief_i2c_error_is_retried_without_full_reconnect(self):
        system = IMURehabSystem(channels=(0, 1))
        system.bus = FlakyBus(failures=2)

        with mock.patch("imu_control.time.sleep"):
            raw = system.read_channel_raw(1)

        self.assertAlmostEqual(raw["ax"], 1.0)
        self.assertEqual(system.transient_i2c_retry_count, 2)
        self.assertEqual(system.reconnect_count, 0)
        self.assertEqual(
            system.bus.channel_writes,
            [(system.TCA_ADDR, 0b10)] * 3,
        )


if __name__ == "__main__":
    unittest.main()
