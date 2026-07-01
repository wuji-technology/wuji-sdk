import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace


def load_device_selection():
    path = Path(__file__).resolve().parents[1] / "examples/python/wuji_glove/_device_selection.py"
    spec = importlib.util.spec_from_file_location("wuji_glove_device_selection", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeGlove:
    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.device_name = None


class FakeHand:
    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.device_name = None


class FakeHand2:
    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.device_name = None


class FakeManager:
    def __init__(self, devices, connected):
        self.devices = devices
        self.connected = connected
        self.connect_calls = []
        self.disconnect_calls = []

    def scan(self):
        return self.devices

    def connect(self, *, sn, device_name, options=None):
        self.connect_calls.append((sn, device_name, options))
        device = self.connected[sn]
        device.device_name = device_name
        return device

    def disconnect(self, device_name):
        self.disconnect_calls.append(device_name)


class WujiGloveDeviceSelectionTest(unittest.TestCase):
    def test_connect_gloves_skips_non_glove_scan_results(self):
        selection = load_device_selection()
        selection.WujiGlove = FakeGlove
        manager = FakeManager(
            [
                SimpleNamespace(sn="WGLOVE", address="192.168.1.101:50001"),
                SimpleNamespace(sn="HAND", address="usb:0483:2000:HAND"),
            ],
            {
                "WGLOVE": FakeGlove("WGLOVE"),
                "HAND": FakeHand("HAND"),
            },
        )

        gloves, scanned = selection.connect_gloves(manager, device_name_prefix="glove")

        self.assertEqual(["WGLOVE"], [glove.serial_number for glove in gloves])
        self.assertEqual(["WGLOVE", "HAND"], [device.sn for device in scanned])
        self.assertEqual(
            [("WGLOVE", "glove_0", None), ("HAND", "glove_1", None)],
            manager.connect_calls,
        )
        self.assertEqual(["glove_1"], manager.disconnect_calls)

    def test_connect_gloves_keeps_multiple_gloves_and_skips_multiple_hand_types(self):
        selection = load_device_selection()
        selection.WujiGlove = FakeGlove
        manager = FakeManager(
            [
                SimpleNamespace(sn="HAND1", address="usb:0483:2000:HAND1"),
                SimpleNamespace(sn="WGLOVE_A", address="192.168.1.101:50001"),
                SimpleNamespace(sn="HAND2", address="usb:0483:2000:HAND2"),
                SimpleNamespace(sn="WGLOVE_B", address="192.168.1.102:50001"),
            ],
            {
                "HAND1": FakeHand("HAND1"),
                "WGLOVE_A": FakeGlove("WGLOVE_A"),
                "HAND2": FakeHand2("HAND2"),
                "WGLOVE_B": FakeGlove("WGLOVE_B"),
            },
        )

        gloves, scanned = selection.connect_gloves(manager, device_name_prefix="glove")

        self.assertEqual(["WGLOVE_A", "WGLOVE_B"], [glove.serial_number for glove in gloves])
        self.assertEqual(["HAND1", "WGLOVE_A", "HAND2", "WGLOVE_B"], [device.sn for device in scanned])
        self.assertEqual(
            [
                ("HAND1", "glove_0", None),
                ("WGLOVE_A", "glove_1", None),
                ("HAND2", "glove_2", None),
                ("WGLOVE_B", "glove_3", None),
            ],
            manager.connect_calls,
        )
        self.assertEqual(["glove_0", "glove_2"], manager.disconnect_calls)

    def test_connect_gloves_rejects_sn_that_is_not_a_glove(self):
        selection = load_device_selection()
        selection.WujiGlove = FakeGlove
        manager = FakeManager(
            [
                SimpleNamespace(sn="WGLOVE", address="192.168.1.101:50001"),
                SimpleNamespace(sn="HAND", address="usb:0483:2000:HAND"),
            ],
            {
                "WGLOVE": FakeGlove("WGLOVE"),
                "HAND": FakeHand("HAND"),
            },
        )

        gloves, scanned = selection.connect_gloves(manager, sn="HAND", device_name_prefix="glove")

        self.assertEqual([], gloves)
        self.assertEqual(["WGLOVE", "HAND"], [device.sn for device in scanned])
        self.assertEqual([("HAND", "glove_0", None)], manager.connect_calls)
        self.assertEqual(["glove_0"], manager.disconnect_calls)

    def test_connect_first_glove_stops_after_first_glove(self):
        selection = load_device_selection()
        selection.WujiGlove = FakeGlove
        manager = FakeManager(
            [
                SimpleNamespace(sn="HAND", address="usb:0483:2000:HAND"),
                SimpleNamespace(sn="WGLOVE", address="192.168.1.101:50001"),
                SimpleNamespace(sn="WGLOVE_B", address="192.168.1.102:50001"),
                SimpleNamespace(sn="HAND2", address="usb:0483:2000:HAND2"),
            ],
            {
                "HAND": FakeHand("HAND"),
                "WGLOVE": FakeGlove("WGLOVE"),
                "WGLOVE_B": FakeGlove("WGLOVE_B"),
                "HAND2": FakeHand2("HAND2"),
            },
        )

        glove, scanned = selection.connect_first_glove(manager, device_name="glove")

        self.assertEqual("WGLOVE", glove.serial_number)
        self.assertEqual(["HAND", "WGLOVE", "WGLOVE_B", "HAND2"], [device.sn for device in scanned])
        self.assertEqual([("HAND", "glove", None), ("WGLOVE", "glove", None)], manager.connect_calls)
        self.assertEqual(["glove"], manager.disconnect_calls)

    def test_connect_first_glove_can_select_second_glove_by_sn(self):
        selection = load_device_selection()
        selection.WujiGlove = FakeGlove
        manager = FakeManager(
            [
                SimpleNamespace(sn="WGLOVE_A", address="192.168.1.101:50001"),
                SimpleNamespace(sn="HAND", address="usb:0483:2000:HAND"),
                SimpleNamespace(sn="WGLOVE_B", address="192.168.1.102:50001"),
            ],
            {
                "WGLOVE_A": FakeGlove("WGLOVE_A"),
                "HAND": FakeHand("HAND"),
                "WGLOVE_B": FakeGlove("WGLOVE_B"),
            },
        )

        glove, scanned = selection.connect_first_glove(manager, sn="WGLOVE_B", device_name="glove")

        self.assertEqual("WGLOVE_B", glove.serial_number)
        self.assertEqual(["WGLOVE_A", "HAND", "WGLOVE_B"], [device.sn for device in scanned])
        self.assertEqual([("WGLOVE_B", "glove", None)], manager.connect_calls)
        self.assertEqual([], manager.disconnect_calls)


if __name__ == "__main__":
    unittest.main()
