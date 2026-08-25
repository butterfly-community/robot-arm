import pathlib
import sys
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).parents[1] / "ros2" / "stararm102_teleop_moveit"
RUN_SCRIPT = pathlib.Path(__file__).parents[1] / "ros2" / "run-moveit-hardware.sh"
sys.path.insert(0, str(PACKAGE_ROOT))
from stararm102_teleop_moveit.gripper_power_mode import (  # noqa: E402
    POWER_PROTECTION_MW,
    STALL_POWER_LIMIT_MW,
    ensure_gripper_power_mode_two,
)


class FakeSerial:
    def __init__(
        self,
        *,
        mode_two: bool,
        corrupt_checksum: bool = False,
        ignore_configuration: bool = False,
    ) -> None:
        self.payload = bytearray(range(33))
        self.payload[0] = 6
        self.payload[6] = 0 if mode_two else 1
        self.payload[7:9] = (STALL_POWER_LIMIT_MW if mode_two else 900).to_bytes(
            2, "little"
        )
        self.payload[15:17] = (
            POWER_PROTECTION_MW if mode_two else 1_000
        ).to_bytes(2, "little")
        self.writes: list[bytes] = []
        self.corrupt_checksum = corrupt_checksum
        self.ignore_configuration = ignore_configuration

    def reset_input_buffer(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        if len(data) == 38 and not self.ignore_configuration:
            self.payload = bytearray(data[4:-1])
        return len(data)

    def read(self, size: int) -> bytes:
        packet = bytes([0x12, 0x4C, 0x06, 0x21]) + self.payload
        checksum = sum(packet) % 256
        response = packet + bytes([checksum ^ int(self.corrupt_checksum)])
        return response[:size]


class GripperPowerModeTests(unittest.TestCase):
    def test_existing_mode_two_is_read_without_rewriting_flash(self):
        transport = FakeSerial(mode_two=True)

        changed = ensure_gripper_power_mode_two(transport)

        self.assertFalse(changed)
        self.assertEqual([len(packet) for packet in transport.writes], [6])

    def test_only_mode_two_fields_are_changed_and_read_back(self):
        transport = FakeSerial(mode_two=False)
        original = bytes(transport.payload)

        changed = ensure_gripper_power_mode_two(transport, settle=lambda _: None)

        self.assertTrue(changed)
        self.assertEqual([len(packet) for packet in transport.writes], [6, 38, 6])
        configuration_packet = transport.writes[1]
        self.assertEqual(configuration_packet[-1], sum(configuration_packet[:-1]) % 256)
        changed_offsets = {6, 7, 8, 15, 16}
        for index, value in enumerate(original):
            if index not in changed_offsets:
                self.assertEqual(transport.payload[index], value)
        self.assertEqual(transport.payload[6], 0)
        self.assertEqual(
            int.from_bytes(transport.payload[7:9], "little"), STALL_POWER_LIMIT_MW
        )
        self.assertEqual(
            int.from_bytes(transport.payload[15:17], "little"), POWER_PROTECTION_MW
        )

    def test_hardware_startup_ensures_mode_two_before_ros_launch(self):
        script = RUN_SCRIPT.read_text()
        configure = script.index("configure_gripper_power_mode")
        launch = script.index("exec ros2 launch")
        self.assertLess(configure, launch)

    def test_corrupt_read_is_rejected_without_configuration_write(self):
        transport = FakeSerial(mode_two=True, corrupt_checksum=True)

        with self.assertRaisesRegex(RuntimeError, "校验和错误"):
            ensure_gripper_power_mode_two(transport)

        self.assertEqual([len(packet) for packet in transport.writes], [6])

    def test_configuration_must_match_on_readback(self):
        transport = FakeSerial(mode_two=False, ignore_configuration=True)

        with self.assertRaisesRegex(RuntimeError, "写入后校验失败"):
            ensure_gripper_power_mode_two(transport, settle=lambda _: None)

        self.assertEqual([len(packet) for packet in transport.writes], [6, 38, 6])


if __name__ == "__main__":
    unittest.main()
