import unittest

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


if __name__ == "__main__":
    unittest.main()
