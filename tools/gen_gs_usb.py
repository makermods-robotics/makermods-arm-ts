#!/usr/bin/env python3
"""Record the Python gs_usb transport's behaviour for the TypeScript port's contract test.

    ~/maker-arm-sdk/.venv/bin/python tools/gen_gs_usb.py     # writes contract/fixtures/gs_usb.json

Runs maker_arm.transport.gs_usb against an in-memory fake device (no USB device is ever touched):
bit timing over a grid of clocks, bitrates and BT_CONST limits; the control transfers and bulk
packets of open, send and close; what receive delivers and counts; port parsing; adapter selection
and the never-acknowledged error message.
"""

import json
import struct
import subprocess
import sys
from collections import deque
from pathlib import Path

import usb.core

from maker_arm.errors import ConnectError
from maker_arm.transport import gs_usb

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "contract" / "fixtures" / "gs_usb.json"
SDK = Path(gs_usb.__file__).resolve().parents[2]
FRAME = struct.Struct("<IIBBBB8s")

CANDLELIGHT = (1, 16, 1, 8, 4, 1, 1024, 1)
LIMITS = {
    "candlelight_f072": CANDLELIGHT,
    "fdcan_g431": (1, 256, 1, 128, 128, 1, 512, 1),
    "brp_step_2": (2, 16, 1, 8, 4, 2, 64, 2),
    "tiny": (1, 4, 1, 2, 1, 1, 1, 1),
}
CLOCKS = [7_000_000, 8_000_000, 16_000_000, 24_000_000, 25_000_000, 36_000_000, 40_000_000, 42_000_000,
          48_000_000, 60_000_000, 64_000_000, 72_000_000, 80_000_000, 84_000_000, 120_000_000,
          160_000_000, 170_000_000]
BITRATES = [10_000, 20_000, 50_000, 83_333, 100_000, 125_000, 250_000, 500_000, 800_000, 1_000_000]


class Ctx:
    def managed_claim_interface(self, dev, interface):
        dev.log.append(["claim", interface])

    def managed_release_interface(self, dev, interface):
        dev.log.append(["release", interface])

    def dispose(self, dev):
        dev.log.append(["dispose"])


class FakeDevice:
    idVendor, idProduct = 0x1D50, 0x606F

    def __init__(self, serial="ABC123", fclk=48_000_000, ack=True):
        self.serial_number = serial
        self._ctx = Ctx()
        self.fclk = fclk
        self.ack = ack
        self.log = []
        self.rx = deque()

    def is_kernel_driver_active(self, interface):
        return False

    def get_active_configuration(self):
        return object()

    def ctrl_transfer(self, bm, req, value, index, data_or_length, timeout):
        if bm == 0xC1:
            self.log.append(["in", req, value, index, data_or_length, timeout])
            return bytearray(struct.pack("<10I", 0x13, self.fclk, *CANDLELIGHT))
        self.log.append(["out", req, value, index, bytes(data_or_length).hex().upper(), timeout])
        return len(data_or_length)

    def write(self, endpoint, data, timeout):
        self.log.append(["write", endpoint, bytes(data).hex().upper()])
        if self.ack:
            echo, ident, dlc, _, _, _, payload = FRAME.unpack(bytes(data))
            self.rx.append(FRAME.pack(echo, ident, dlc, 0, 0, 0, payload))
        return len(data)

    def read(self, endpoint, size, timeout):
        try:
            return bytearray(self.rx.popleft())
        except IndexError:
            import time
            time.sleep(0.0005)
            raise usb.core.USBTimeoutError("timeout")


def bit_timings():
    out = []
    for name, limits in LIMITS.items():
        for fclk in CLOCKS:
            for bitrate in BITRATES:
                case = {"limits": name, "fclk": fclk, "bitrate": bitrate}
                try:
                    case["timing"] = list(gs_usb.bit_timing(fclk, bitrate, *limits))
                except ConnectError as e:
                    case["error"] = str(e)
                out.append(case)
    return out


SENDS = [
    (0x7FF, "0700CC0000000000", False),
    (0x000, "", False),
    (0x123, "01", False),
    (0x1234567, "0102", True),
    (0x1FFFFFFF, "FFFFFFFFFFFFFFFF", True),
    (0x12345, "AABBCC", False),          # standard: masked to 11 bits
    (0x3FFFFFFFF & 0xFFFFFFFF, "11223344556677", True),   # extended: masked to 29 bits
]


def lifecycle():
    dev = FakeDevice()
    backend = gs_usb.GsUsbBackend("gs_usb:" + dev.serial_number, device=dev)
    backend.open()
    opened = list(dev.log)
    dev.log.clear()
    for can_id, data, extended in SENDS:
        backend.send(can_id, bytes.fromhex(data), extended=extended)
    import time
    time.sleep(0.05)
    sent = [e[2] for e in dev.log if e[0] == "write"]
    dev.log.clear()
    backend.close()
    closed = list(dev.log)
    return {
        "open": opened,
        "sends": [{"id": c, "data": d, "extended": x, "bytes": b} for (c, d, x), b in zip(SENDS, sent)],
        "close": closed,
        "bitrate_500k_open": _open_at(500_000),
        "fclk_80mhz_open": _open_at(1_000_000, fclk=80_000_000),
        "io_timeout_0_25_open": _open_at(1_000_000, io_timeout=0.25),
    }


def _open_at(bitrate, fclk=48_000_000, io_timeout=0.1):
    dev = FakeDevice(fclk=fclk)
    backend = gs_usb.GsUsbBackend("gs_usb:X", device=dev, bitrate=bitrate, io_timeout=io_timeout)
    backend.open()
    backend.close()
    return dev.log


def rx(echo, ident, dlc, channel, flags, data):
    return FRAME.pack(echo, ident, dlc, channel, flags, 0, bytes.fromhex(data).ljust(8, b"\x00")).hex().upper()


RECEIVES = [
    ("standard", rx(0xFFFFFFFF, 0x17, 8, 0, 0, "0001020304050607")),
    ("extended", rx(0xFFFFFFFF, 0x1ABCDEF | gs_usb.CAN_EFF_FLAG, 1, 0, 0, "AA")),
    ("echo", rx(3, 0x11, 8, 0, 0, "")),
    ("echo_with_flags", rx(4, 0x11, 8, 1, 0x02, "")),
    ("error_frame", rx(0xFFFFFFFF, gs_usb.CAN_ERR_FLAG | 4, 8, 0, 0, "0000100000000000")),
    ("error_frame_overflow", rx(0xFFFFFFFF, gs_usb.CAN_ERR_FLAG | 4, 8, 0, 1, "")),
    ("other_channel", rx(0xFFFFFFFF, 0x12, 8, 1, 0, "")),
    ("fd_flag", rx(0xFFFFFFFF, 0x13, 8, 0, 0x02, "")),
    ("overflow", rx(0xFFFFFFFF, 0x14, 1, 0, 0x01, "01")),
    ("overflow_other_channel", rx(0xFFFFFFFF, 0x14, 1, 2, 0x01, "01")),
    ("rtr", rx(0xFFFFFFFF, 0x15 | gs_usb.CAN_RTR_FLAG, 0, 0, 0, "")),
    ("rtr_extended", rx(0xFFFFFFFF, 0x15 | gs_usb.CAN_RTR_FLAG | gs_usb.CAN_EFF_FLAG, 0, 0, 0, "")),
    ("dlc_0", rx(0xFFFFFFFF, 0x16, 0, 0, 0, "")),
    ("dlc_15", rx(0xFFFFFFFF, 0x16, 15, 0, 0, "0102030405060708")),
    ("standard_high_bits", rx(0xFFFFFFFF, 0x0FFFF, 2, 0, 0, "0102")),
    ("short", "00" * 19),
    ("hw_timestamp_24", rx(0xFFFFFFFF, 0x18, 2, 0, 0, "BEEF") + "01020304"),
    ("two_frames", rx(0xFFFFFFFF, 0x19, 1, 0, 0, "01") + rx(0xFFFFFFFF, 0x1A, 1, 0, 0, "02")),
]


def receives():
    out = []
    for name, raw in RECEIVES:
        backend = gs_usb.GsUsbBackend("gs_usb:X", device=FakeDevice())
        got = []
        backend.set_recv_callback(lambda cid, data: got.append([int(cid), cid.extended, bytes(data).hex().upper()]))
        backend._handle(bytes.fromhex(raw))
        out.append({"case": name, "bytes": raw, "received": got, "error_frames": backend.error_frames,
                    "rx_overflows": backend.rx_overflows})
    return out


def parse_ports():
    out = []
    for port in [None, "", "gs_usb", "gs_usb:", "gs_usb:ABC", "ABC", "207A37AA45465006", "gs_usb:A B",
                 "A\tB", "gs_usb:gs_usb:X"]:
        try:
            out.append({"port": port, "serial": gs_usb.parse_port(port)})
        except ValueError as e:
            out.append({"port": port, "error": str(e)})
    return out


def selection():
    out = []
    for serials, port in [(["A1", "B2"], "gs_usb:B2"), (["A1", "B2"], None), (["A1", "B2"], "C3"),
                          (["A1"], None), ([], None), ([], "gs_usb:A1"), (["A1", "A1"], "A1")]:
        devices = [FakeDevice(s) for s in serials]
        original = gs_usb._find_devices
        gs_usb._find_devices = lambda devices=devices: devices
        try:
            backend = gs_usb.GsUsbBackend(port)
            try:
                chosen = backend._select()
                out.append({"serials": serials, "port": port, "chosen": devices.index(chosen)})
            except ConnectError as e:
                out.append({"serials": serials, "port": port, "error": str(e)})
        finally:
            gs_usb._find_devices = original
    return out


def never_acknowledged():
    dev = FakeDevice(ack=False)
    backend = gs_usb.GsUsbBackend("gs_usb:ABC123", device=dev, io_timeout=0.02)
    backend.open()
    try:
        for _ in range(gs_usb.MAX_IN_FLIGHT):
            backend.send(1, bytes(8), extended=False)
        try:
            backend.send(1, bytes(8), extended=False)
        except TimeoutError as e:
            return {"message": str(e), "writes": sum(1 for e in dev.log if e[0] == "write")}
    finally:
        backend.close()
    raise AssertionError("expected TimeoutError")


def main() -> int:
    commit = subprocess.run(["git", "-C", str(SDK), "rev-parse", "--short", "HEAD"], capture_output=True,
                            text=True).stdout.strip()
    data = {
        "meta": {"generated_by": "tools/gen_gs_usb.py", "source": "maker_arm/transport/gs_usb.py",
                 "sdk_commit": commit, "python": sys.version.split()[0]},
        "constants": {"MAX_IN_FLIGHT": gs_usb.MAX_IN_FLIGHT, "EP_OUT": gs_usb.EP_OUT, "EP_IN": gs_usb.EP_IN,
                      "ids": sorted([list(p) for p in gs_usb.GS_USB_IDS])},
        "limits": {k: list(v) for k, v in LIMITS.items()},
        "bit_timing": bit_timings(),
        "lifecycle": lifecycle(),
        "receives": receives(),
        "parse_port": parse_ports(),
        "selection": selection(),
        "never_acknowledged": never_acknowledged(),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=1) + "\n")
    print(f"wrote {OUT.relative_to(ROOT)}: {len(data['bit_timing'])} bit timings, {len(data['receives'])} receives")
    return 0


if __name__ == "__main__":
    sys.exit(main())
