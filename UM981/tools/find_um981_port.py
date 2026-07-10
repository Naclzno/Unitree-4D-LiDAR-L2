#!/usr/bin/env python3
from __future__ import annotations

import glob
import os
import subprocess
from dataclasses import dataclass


TTY_PATTERNS = [
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
]


@dataclass(frozen=True)
class SerialDevice:
    path: str
    by_id: tuple[str, ...]
    properties: tuple[tuple[str, str], ...]


def list_tty_paths() -> list[str]:
    paths: set[str] = set()
    for pattern in TTY_PATTERNS:
        paths.update(glob.glob(pattern))
    return sorted(paths)


def list_by_id_for(path: str) -> tuple[str, ...]:
    result: list[str] = []
    real_path = os.path.realpath(path)
    for item in sorted(glob.glob("/dev/serial/by-id/*")):
        if os.path.realpath(item) == real_path:
            result.append(item)
    return tuple(result)


def udev_properties(path: str) -> tuple[tuple[str, str], ...]:
    keys = {
        "DEVNAME",
        "ID_VENDOR",
        "ID_VENDOR_ID",
        "ID_MODEL",
        "ID_MODEL_ID",
        "ID_SERIAL",
        "ID_SERIAL_SHORT",
        "ID_USB_DRIVER",
        "ID_USB_INTERFACES",
    }
    try:
        output = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", path],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return tuple()

    props: list[tuple[str, str]] = []
    for line in output.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key in keys:
            props.append((key, value))
    return tuple(props)


def snapshot() -> dict[str, SerialDevice]:
    out: dict[str, SerialDevice] = {}
    for path in list_tty_paths():
        out[path] = SerialDevice(
            path=path,
            by_id=list_by_id_for(path),
            properties=udev_properties(path),
        )
    return out


def print_snapshot(title: str, devices: dict[str, SerialDevice]) -> None:
    print(f"\n## {title}")
    if not devices:
        print("(no /dev/ttyUSB* or /dev/ttyACM* devices found)")
        return
    for path, device in devices.items():
        print(f"\n=== {path} ===")
        if device.by_id:
            print("by-id:")
            for item in device.by_id:
                print(f"  {item}")
        if device.properties:
            print("udev:")
            for key, value in device.properties:
                print(f"  {key}={value}")


def main() -> int:
    print("Step 1: keep UM981 unplugged. Existing serial devices will be recorded.")
    before = snapshot()
    print_snapshot("Before plugging UM981", before)

    input("\nNow plug in UM981, wait 2 seconds, then press Enter...")

    after = snapshot()
    print_snapshot("After plugging UM981", after)

    before_paths = set(before)
    after_paths = set(after)
    added = sorted(after_paths - before_paths)
    removed = sorted(before_paths - after_paths)

    print("\n## Difference")
    if added:
        print("New serial device(s), likely UM981:")
        for path in added:
            device = after[path]
            print(f"\n  {path}")
            if device.by_id:
                print("  stable by-id path:")
                for item in device.by_id:
                    print(f"    {item}")
            serial = dict(device.properties).get("ID_SERIAL")
            model = dict(device.properties).get("ID_MODEL")
            if serial or model:
                print(f"  model={model or '(unknown)'} serial={serial or '(unknown)'}")
    else:
        print("No new /dev/ttyUSB* or /dev/ttyACM* device appeared.")
        print("If UM981 is powered through a USB-to-UART bridge already present before plugging,")
        print("check wiring TX/RX/GND and repeat this test while unplugging the USB bridge itself.")

    if removed:
        print("\nRemoved device(s):")
        for path in removed:
            print(f"  {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
