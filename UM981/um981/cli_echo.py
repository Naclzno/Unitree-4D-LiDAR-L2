from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path
from typing import TextIO

from .sdk import DEFAULT_COMMANDS, Um981Serial


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read UM981 serial data and print decoded observations as JSON lines.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial device, usually /dev/ttyUSB0 after usbipd attach")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--timeout", type=float, default=0.2, help="serial read timeout seconds")
    parser.add_argument("--read-size", type=int, default=4096, help="serial read chunk size")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to run; 0 means until Ctrl+C")
    parser.add_argument("--log-dir", default="logs", help="directory for timestamped txt logs")
    parser.add_argument("--no-log", action="store_true", help="print only; do not save txt logs")
    parser.add_argument(
        "--command",
        action="append",
        default=[],
        help="receiver command sent after opening; can be repeated",
    )
    parser.add_argument(
        "--no-default-commands",
        action="store_true",
        help="do not send the default RAWIMUXA output command",
    )
    parser.add_argument(
        "--type",
        action="append",
        choices=["position", "imu"],
        help="print only selected observation type; can be repeated",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    commands: list[str] = []
    if not args.no_default_commands:
        commands.extend(DEFAULT_COMMANDS)
    commands.extend(args.command)
    selected_types = set(args.type or [])
    log_files: dict[str, TextIO] = {}
    if not args.no_log:
        log_root = Path(args.log_dir)
        position_dir = log_root / "positions"
        imu_dir = log_root / "imus"
        position_dir.mkdir(parents=True, exist_ok=True)
        imu_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        position_path = position_dir / f"um981_position_{stamp}.txt"
        imu_path = imu_dir / f"um981_imu_{stamp}.txt"
        if not selected_types or "position" in selected_types:
            log_files["position"] = position_path.open("w", encoding="utf-8")
            print(f"logging positions to {position_path}", file=sys.stderr)
        if not selected_types or "imu" in selected_types:
            log_files["imu"] = imu_path.open("w", encoding="utf-8")
            print(f"logging imus to {imu_path}", file=sys.stderr)

    try:
        with Um981Serial(
            port=args.port,
            baud=args.baud,
            timeout=args.timeout,
            read_size=args.read_size,
            commands=commands,
        ) as receiver:
            for observation in receiver.iter_observations(
                duration=args.duration,
            ):
                if selected_types and observation.type not in selected_types:
                    continue
                line = json.dumps(observation.to_dict(), ensure_ascii=False)
                print(line, flush=True)
                log_file = log_files.get(observation.type)
                if log_file is not None:
                    log_file.write(line + "\n")
                    log_file.flush()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"um981 echo failed: {exc}", file=sys.stderr)
        return 2
    finally:
        for log_file in log_files.values():
            log_file.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
