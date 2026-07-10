from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class GnssFix:
    lat_deg: float | None
    lon_deg: float | None
    height_m: float | None = None
    fix_quality: str | None = None
    satellites_used: int | None = None
    hdop: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    heading_deg: float | None = None
    roll_sigma_deg: float | None = None
    pitch_sigma_deg: float | None = None
    heading_sigma_deg: float | None = None
    ins_status: str | int | None = None
    source: str = "gnss"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ImuSample:
    week: int | None
    seconds: float | None
    accel_mps2: Vector3
    gyro_radps: Vector3
    imu_error: str | int | None = None
    imu_type: int | None = None
    imu_status: str | int | None = None
    raw_counts: list[int] | None = None

    def to_dict(self) -> dict[str, Any]:
        out = asdict(self)
        out["accel_mps2"] = self.accel_mps2.to_dict()
        out["gyro_radps"] = self.gyro_radps.to_dict()
        return out


@dataclass(frozen=True)
class Observation:
    type: str
    payload: GnssFix | ImuSample | dict[str, Any]
    message: str | None = None
    raw_text: str | None = None
    parsed: dict[str, Any] | None = None

    def to_dict(self) -> dict[str, Any]:
        if hasattr(self.payload, "to_dict"):
            payload = self.payload.to_dict()  # type: ignore[union-attr]
        else:
            payload = self.payload
        return {
            "type": self.type,
            "message": self.message,
            "payload": payload,
            "raw_text": self.raw_text,
        }


def dps_to_radps(value: float) -> float:
    return value * math.pi / 180.0
