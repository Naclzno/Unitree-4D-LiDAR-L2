#!/usr/bin/env python3
from __future__ import annotations

import argparse
import string
import time


DEFAULT_BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000]
DEFAULT_COMMANDS = [
    "RAWIMUXA 0.01",
    "IMUATTA 0.1",
    "GYRATTA 0.1",
    "INSPVAXA 0.1",
    "GNGGA 1",
]


def printable(data: bytes) -> str:
    allowed = set(string.printable)
    text = data.decode("ascii", errors="replace")
    return "".join(ch if ch in allowed else "." for ch in text)


def probe(port: str, baud: int, commands: list[str], wait: float, read_size: int) -> None:
    import serial

    print(f"\n=== {port} @ {baud} ===", flush=True)
    try:
        with serial.Serial(port, baud, timeout=wait) as serial_port:
            serial_port.reset_input_buffer()
            serial_port.reset_output_buffer()

            before = serial_port.read(read_size)
            if before:
                print(f"passive bytes={len(before)}")
                print("ascii:", printable(before[:500]))
                print("hex:  ", before[:120].hex(" "))

            for command in commands:
                payload = (command.strip() + "\r\n").encode("ascii")
                print(f"send: {command}")
                serial_port.write(payload)
                time.sleep(wait)
                data = serial_port.read(read_size)
                print(f"bytes={len(data)}")
                if data:
                    print("ascii:", printable(data[:500]))
                    print("hex:  ", data[:120].hex(" "))
    except Exception as exc:
        print(f"error: {exc}")


def listen(port: str, baud: int, commands: list[str], duration: float, read_size: int) -> None:
    import serial

    print(f"\n=== continuous listen {port} @ {baud}, {duration:.1f}s ===", flush=True)
    try:
        with serial.Serial(port, baud, timeout=0.2) as serial_port:
            serial_port.reset_input_buffer()
            serial_port.reset_output_buffer()
            for command in commands:
                print(f"send: {command}")
                serial_port.write((command.strip() + "\r\n").encode("ascii"))
                time.sleep(0.2)

            deadline = time.monotonic() + duration
            total = bytearray()
            while time.monotonic() < deadline:
                chunk = serial_port.read(read_size)
                if chunk:
                    total.extend(chunk)

            print(f"total bytes={len(total)}")
            if total:
                print("ascii:")
                print(printable(bytes(total[:3000])))
                print("hex:")
                print(bytes(total[:300]).hex(" "))
                for marker in [b"#RAWIMUX", b"#IMUATT", b"#GYRATT", b"#INSPVAX", b"$GNGGA", b"$command"]:
                    print(f"{marker.decode('ascii')}: {total.count(marker)}")
    except Exception as exc:
        print(f"error: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe UM981 serial baud rate and raw output.")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", action="append", type=int, default=[])
    parser.add_argument("--command", action="append", default=[])
    parser.add_argument("--wait", type=float, default=1.0)
    parser.add_argument("--listen", type=float, default=0.0, help="after probing, listen this many seconds at the first baud")
    parser.add_argument("--read-size", type=int, default=4096)
    args = parser.parse_args()

    bauds = args.baud or DEFAULT_BAUDS
    commands = args.command or DEFAULT_COMMANDS
    for baud in bauds:
        probe(args.port, baud, commands, args.wait, args.read_size)
    if args.listen > 0.0:
        listen(args.port, bauds[0], commands, args.listen, args.read_size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
