from __future__ import annotations

import time
from collections.abc import Iterator
from typing import Any

from .models import GnssFix, ImuSample, Observation, Vector3, dps_to_radps
from .protocol import Frame, decode_frame, extract_frames


DEFAULT_COMMANDS = (
    "RAWIMUXA 0.01",
    "IMUATTA 0.1",
    "INSPVAXA 0.1",
)


class Um981Serial:
    """Serial reader for UM981 receivers.

    The class intentionally does not depend on ROS. Use it directly in WSL2 for
    SDK testing, then reuse the same class inside a ROS node on the Ubuntu host.
    """

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 0.2,
        read_size: int = 4096,
        commands: list[str] | tuple[str, ...] = DEFAULT_COMMANDS,
    ) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.read_size = read_size
        self.commands = tuple(commands)
        self._serial: Any | None = None
        self._buffer = bytearray()

    def open(self) -> None:
        try:
            import serial
        except ModuleNotFoundError as exc:
            raise RuntimeError("missing dependency pyserial; install with: python3 -m pip install pyserial") from exc

        self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        for command in self.commands:
            self.send_command(command)
            time.sleep(0.1)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "Um981Serial":
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    def send_command(self, command: str) -> None:
        if self._serial is None:
            raise RuntimeError("serial port is not open")
        cmd = command.strip()
        if not cmd:
            return
        self._serial.write((cmd + "\r\n").encode("ascii"))

    def iter_frames(self, duration: float = 0.0) -> Iterator[Frame]:
        if self._serial is None:
            raise RuntimeError("serial port is not open")

        started = time.monotonic()
        while True:
            if duration and time.monotonic() - started >= duration:
                break
            chunk = self._serial.read(self.read_size)
            if not chunk:
                continue
            self._buffer.extend(chunk)
            yield from extract_frames(self._buffer)

    def iter_decoded(self, duration: float = 0.0) -> Iterator[dict[str, Any]]:
        for frame in self.iter_frames(duration=duration):
            yield decode_frame(frame)

    def iter_observations(self, duration: float = 0.0, include_unknown: bool = False) -> Iterator[Observation]:
        for frame in self.iter_frames(duration=duration):
            decoded = decode_frame(frame)
            observation = observation_from_decoded(decoded, raw_text=frame.text)
            if observation is not None:
                yield observation
            elif include_unknown:
                yield Observation(
                    type=decoded.get("type", "unknown"),
                    payload=decoded,
                    message=decoded.get("message"),
                    raw_text=frame.text,
                    parsed=decoded,
                )


def observation_from_decoded(decoded: dict[str, Any], raw_text: str | None = None) -> Observation | None:
    message = decoded.get("message")
    data = decoded.get("decoded")
    if not isinstance(data, dict):
        return None

    if decoded.get("type") == "nmea" and str(message).endswith("GGA"):
        fix = GnssFix(
            lat_deg=data.get("lat_deg"),
            lon_deg=data.get("lon_deg"),
            height_m=data.get("altitude_m"),
            fix_quality=data.get("fix_quality"),
            satellites_used=data.get("satellites_used"),
            hdop=data.get("hdop"),
            source=str(message),
        )
        return Observation(type="position", payload=fix, message=message, raw_text=raw_text, parsed=decoded)

    base_message = str(message or "").rstrip("ABS")

    if base_message == "RAWIMUX":
        imu = imu_from_rawimux(data)
        return Observation(type="imu", payload=imu, message=message, raw_text=raw_text, parsed=decoded)

    if base_message == "IMUATT":
        imu = imu_from_imuatt(data, decoded)
        return Observation(type="imu", payload=imu, message=message, raw_text=raw_text, parsed=decoded)

    if base_message == "INSPVAX":
        fix = fix_from_inspvax(data, message)
        if valid_lat_lon(fix.lat_deg, fix.lon_deg):
            return Observation(type="position", payload=fix, message=message, raw_text=raw_text, parsed=decoded)
        return None

    if base_message == "DRPVA":
        fix = fix_from_drpva(data, message)
        if valid_lat_lon(fix.lat_deg, fix.lon_deg):
            return Observation(type="position", payload=fix, message=message, raw_text=raw_text, parsed=decoded)
        return None

    return None


def imu_from_rawimux(data: dict[str, Any]) -> ImuSample:
    # RAWIMUX reports the sensor axes as z, -y, x. Convert to x, y, z here.
    accel = Vector3(
        x=float(data.get("x_accel_mps2", 0.0)),
        y=-float(data.get("neg_y_accel_mps2", 0.0)),
        z=float(data.get("z_accel_mps2", 0.0)),
    )
    gyro = Vector3(
        x=dps_to_radps(float(data.get("x_gyro_dps", 0.0))),
        y=-dps_to_radps(float(data.get("neg_y_gyro_dps", 0.0))),
        z=dps_to_radps(float(data.get("z_gyro_dps", 0.0))),
    )
    return ImuSample(
        week=data.get("week"),
        seconds=data.get("seconds"),
        accel_mps2=accel,
        gyro_radps=gyro,
        imu_error=data.get("imu_error"),
        imu_type=data.get("imu_type"),
        imu_status=data.get("imu_status_hex", data.get("imu_status")),
        raw_counts=data.get("raw"),
    )


def imu_from_imuatt(data: dict[str, Any], decoded: dict[str, Any]) -> ImuSample:
    accel = Vector3(
        x=float(data.get("x_accel_mps2", 0.0)),
        y=float(data.get("y_accel_mps2", 0.0)),
        z=float(data.get("z_accel_mps2", 0.0)),
    )
    gyro = Vector3(
        x=dps_to_radps(float(data.get("x_gyro_dps", 0.0))),
        y=dps_to_radps(float(data.get("y_gyro_dps", 0.0))),
        z=dps_to_radps(float(data.get("z_gyro_dps", 0.0))),
    )
    return ImuSample(
        week=decoded.get("week"),
        seconds=decoded.get("seconds"),
        accel_mps2=accel,
        gyro_radps=gyro,
        imu_status=data.get("ins_status"),
        raw_counts=data.get("raw"),
    )


def fix_from_inspvax(data: dict[str, Any], message: object) -> GnssFix:
    return GnssFix(
        lat_deg=data.get("lat_deg"),
        lon_deg=data.get("lon_deg"),
        height_m=data.get("height_m"),
        fix_quality=str(data.get("pos_type")) if data.get("pos_type") is not None else None,
        roll_deg=data.get("roll_deg"),
        pitch_deg=data.get("pitch_deg"),
        heading_deg=data.get("azimuth_deg"),
        roll_sigma_deg=data.get("roll_sigma_deg"),
        pitch_sigma_deg=data.get("pitch_sigma_deg"),
        heading_sigma_deg=data.get("azimuth_sigma_deg"),
        ins_status=data.get("ins_status"),
        source=str(message or "INSPVAX"),
    )


def fix_from_drpva(data: dict[str, Any], message: object) -> GnssFix:
    return GnssFix(
        lat_deg=data.get("lat_deg"),
        lon_deg=data.get("lon_deg"),
        height_m=data.get("height_m"),
        fix_quality=str(data.get("pos_type")) if data.get("pos_type") is not None else None,
        roll_deg=data.get("roll_deg"),
        pitch_deg=data.get("pitch_deg"),
        heading_deg=data.get("heading_deg"),
        roll_sigma_deg=data.get("roll_sigma_deg"),
        pitch_sigma_deg=data.get("pitch_sigma_deg"),
        heading_sigma_deg=data.get("heading_sigma_deg"),
        ins_status=data.get("ins_status") or data.get("sol_status"),
        source=str(message or "DRPVA"),
    )


def valid_lat_lon(lat: float | None, lon: float | None) -> bool:
    if lat is None or lon is None:
        return False
    return not (abs(float(lat)) < 1e-12 and abs(float(lon)) < 1e-12)
