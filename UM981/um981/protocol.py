from __future__ import annotations

import binascii
import struct
from dataclasses import dataclass
from typing import Any


SYNC_NORMAL = b"\xaa\x44\x12"
SYNC_SHORT = b"\xaa\x44\xb6"

MSG_NAMES = {
    1441: "INSHOTINFOR0",
    1442: "IMUATT",
    1443: "GYRATTS",
    1444: "GYRATT",
    1461: "RAWIMUX",
    1465: "INSPVAX",
    57024: "DRPVA",
}

INS_STATUS = {
    0: "INS_INACTIVE",
    1: "INS_ALIGNING",
    2: "INS_HIGH_VARIANCE",
    3: "INS_SOLUTION_GOOD",
    6: "INS_SOLUTION_FREE",
    7: "INS_ALIGNMENT_COMPLETE",
}

POS_TYPE = {
    0: "NONE",
    52: "INS",
    53: "INS_PSRSP",
    54: "INS_PSRDIFF",
    55: "INS_RTKFLOAT",
    56: "INS_RTKFIXED",
}

SOL_STATUS = {
    0: "SOL_COMPUTED",
    1: "INSUFFICIENT_OBS",
    2: "NO_CONVERGENCE",
    4: "COV_TRACE",
}


@dataclass
class Frame:
    kind: str
    raw: bytes
    text: str | None = None
    msg_id: int | None = None
    header_len: int | None = None
    body: bytes | None = None


def clean_ascii(data: bytes) -> str:
    return data.decode("ascii", errors="replace").strip()


def crc32_um981(payload: bytes) -> int:
    return binascii.crc32(payload) & 0xFFFFFFFF


def s(body: list[str], idx: int, default: str = "") -> str:
    try:
        return body[idx]
    except IndexError:
        return default


def f(body: list[str], idx: int, default: float = 0.0) -> float:
    try:
        return float(body[idx])
    except (IndexError, ValueError):
        return default


def i(body: list[str], idx: int, default: int | None = None) -> int | None:
    try:
        return int(float(body[idx]))
    except (IndexError, ValueError):
        return default


def nmea_lat_lon(value: str, hemi: str) -> float | None:
    if not value:
        return None
    try:
        dot = value.find(".")
        deg_len = dot - 2 if dot >= 0 else len(value) - 2
        degrees = int(value[:deg_len])
        minutes = float(value[deg_len:])
        result = degrees + minutes / 60.0
        if hemi in {"S", "W"}:
            result = -result
        return result
    except ValueError:
        return None


def decode_nmea(text: str) -> dict[str, Any]:
    payload = text[1:]
    checksum = None
    if "*" in payload:
        payload, checksum = payload.split("*", 1)

    talker_msg = payload.split(",", 1)[0]
    fields = payload.split(",")
    out: dict[str, Any] = {"type": "nmea", "message": talker_msg, "fields": fields}
    if checksum is not None:
        value = 0
        for ch in payload.encode("ascii", errors="ignore"):
            value ^= ch
        try:
            expected = int(checksum[:2], 16)
            out["checksum"] = "ok" if expected == value else f"bad expected={expected:02x} actual={value:02x}"
        except ValueError:
            out["checksum"] = "invalid"

    if talker_msg.endswith("GGA"):
        out["decoded"] = decode_nmea_gga(fields)
    elif talker_msg.endswith("RMC"):
        out["decoded"] = decode_nmea_rmc(fields)
    return out


def decode_nmea_gga(fields: list[str]) -> dict[str, Any]:
    fix_quality = {
        "0": "invalid",
        "1": "gps_fix",
        "2": "dgps_fix",
        "4": "rtk_fixed",
        "5": "rtk_float",
    }.get(s(fields, 6), s(fields, 6))
    return {
        "utc": s(fields, 1),
        "lat_deg": nmea_lat_lon(s(fields, 2), s(fields, 3)),
        "lon_deg": nmea_lat_lon(s(fields, 4), s(fields, 5)),
        "fix_quality": fix_quality,
        "satellites_used": i(fields, 7),
        "hdop": f(fields, 8) if s(fields, 8) else None,
        "altitude_m": f(fields, 9) if s(fields, 9) else None,
    }


def decode_nmea_rmc(fields: list[str]) -> dict[str, Any]:
    return {
        "utc": s(fields, 1),
        "status": "valid" if s(fields, 2) == "A" else "invalid",
        "lat_deg": nmea_lat_lon(s(fields, 3), s(fields, 4)),
        "lon_deg": nmea_lat_lon(s(fields, 5), s(fields, 6)),
        "speed_knots": f(fields, 7) if s(fields, 7) else None,
        "course_deg": f(fields, 8) if s(fields, 8) else None,
        "date_ddmmyy": s(fields, 9),
    }


def parse_ascii_sentence(text: str) -> dict[str, Any]:
    if not text or text[0] not in "#%$":
        return {"type": "unknown_ascii", "text": text}

    if text[0] == "$":
        return decode_nmea(text)

    sentence = text[1:]
    crc_text = None
    if "*" in sentence:
        sentence, crc_text = sentence.rsplit("*", 1)

    if ";" in sentence:
        header_text, body_text = sentence.split(";", 1)
    else:
        header_text, body_text = sentence, ""

    header = header_text.split(",") if header_text else []
    body = body_text.split(",") if body_text else []
    msg = header[0] if header else ""
    base_msg = msg.rstrip("ABS")
    out: dict[str, Any] = {
        "type": "um981_ascii",
        "message": msg,
        "header": header,
        "body": body,
        "week": i(header, 5),
        "seconds": f(header, 6) if s(header, 6) else None,
    }

    if crc_text:
        try:
            expected = int(crc_text[:8], 16)
            actual = crc32_um981(sentence.encode("ascii", errors="ignore"))
            out["crc"] = "ok" if expected == actual else f"bad expected={expected:08x} actual={actual:08x}"
        except ValueError:
            out["crc"] = "invalid"

    if base_msg == "INSPVAX":
        out["decoded"] = decode_ascii_inspvax(body)
    elif base_msg == "RAWIMUX":
        out["decoded"] = decode_ascii_rawimux(body)
    elif base_msg == "IMUATT":
        out["decoded"] = decode_ascii_imuatt(body)
    elif base_msg in {"GYRATT", "GYRATTS"}:
        out["decoded"] = decode_ascii_gyratt(body)
    elif base_msg == "DRPVA":
        out["decoded"] = decode_ascii_drpva(body)

    return out


def decode_ascii_inspvax(body: list[str]) -> dict[str, Any]:
    return {
        "ins_status": s(body, 0),
        "pos_type": s(body, 1),
        "lat_deg": f(body, 2),
        "lon_deg": f(body, 3),
        "height_m": f(body, 4),
        "undulation_m": f(body, 5),
        "north_vel_mps": f(body, 6),
        "east_vel_mps": f(body, 7),
        "up_vel_mps": f(body, 8),
        "roll_deg": f(body, 9),
        "pitch_deg": f(body, 10),
        "azimuth_deg": f(body, 11),
        "lat_sigma_m": f(body, 12),
        "lon_sigma_m": f(body, 13),
        "height_sigma_m": f(body, 14),
    }


def decode_ascii_rawimux(body: list[str]) -> dict[str, Any]:
    imu_type = i(body, 1, -1)
    accel_scale = 2.0 * 9.80665 / 32767.0 if imu_type == 64 else None
    gyro_scale = 250.0 / 32767.0 if imu_type == 64 else None
    raw = [int(f(body, idx)) for idx in range(5, min(len(body), 11))]
    decoded: dict[str, Any] = {
        "imu_error": s(body, 0),
        "imu_type": imu_type,
        "week": i(body, 2),
        "seconds": f(body, 3),
        "imu_status_hex": s(body, 4),
        "raw": raw,
    }
    if len(raw) == 6 and accel_scale and gyro_scale:
        decoded.update(
            {
                "z_accel_mps2": raw[0] * accel_scale,
                "neg_y_accel_mps2": raw[1] * accel_scale,
                "x_accel_mps2": raw[2] * accel_scale,
                "z_gyro_dps": raw[3] * gyro_scale,
                "neg_y_gyro_dps": raw[4] * gyro_scale,
                "x_gyro_dps": raw[5] * gyro_scale,
            }
        )
    return decoded


def decode_ascii_imuatt(body: list[str]) -> dict[str, Any]:
    angle_scale = 360.0 / 32767.0
    accel_scale = 80.0 / 32767.0
    gyro_scale = 500.0 / 32767.0
    roll = f(body, 4)
    pitch = f(body, 5)
    azimuth = f(body, 6)
    acc_x = f(body, 7)
    acc_y = f(body, 8)
    acc_z = f(body, 9)
    gyro_x = f(body, 10)
    gyro_y = f(body, 11)
    gyro_z = f(body, 12)
    return {
        "ins_status": s(body, 0),
        "pos_type": s(body, 1),
        "sol_age_0p001s": f(body, 2),
        "dr_age_0p1s": f(body, 3),
        "roll_deg": roll * angle_scale,
        "pitch_deg": pitch * angle_scale,
        "azimuth_deg": azimuth * angle_scale,
        "x_accel_mps2": acc_x * accel_scale,
        "y_accel_mps2": acc_y * accel_scale,
        "z_accel_mps2": acc_z * accel_scale,
        "x_gyro_dps": gyro_x * gyro_scale,
        "y_gyro_dps": gyro_y * gyro_scale,
        "z_gyro_dps": gyro_z * gyro_scale,
        "raw": [int(acc_x), int(acc_y), int(acc_z), int(gyro_x), int(gyro_y), int(gyro_z)],
    }


def decode_ascii_gyratt(body: list[str]) -> dict[str, Any]:
    return {
        "ins_status": s(body, 0),
        "pos_type": s(body, 1),
        "dr_age_0p1s": f(body, 2),
        "sol_age_0p001s": f(body, 3),
        "roll_deg": f(body, 4),
        "pitch_deg": f(body, 5),
        "azimuth_deg": f(body, 6),
        "gyro_z_dps": f(body, 7),
    }


def decode_ascii_drpva(body: list[str]) -> dict[str, Any]:
    return {
        "sol_status": s(body, 0),
        "pos_type": s(body, 1),
        "datum": s(body, 2),
        "dr_age_s": f(body, 7),
        "sol_age_s": f(body, 8),
        "lat_deg": f(body, 9),
        "lon_deg": f(body, 10),
        "height_m": f(body, 11),
        "east_vel_mps": f(body, 16),
        "north_vel_mps": f(body, 17),
        "up_vel_mps": f(body, 18),
        "heading_deg": f(body, 22),
        "pitch_deg": f(body, 23),
        "roll_deg": f(body, 24),
    }


def decode_binary(frame: Frame) -> dict[str, Any]:
    if frame.body is None or frame.msg_id is None:
        return {"type": "binary", "error": "missing body"}

    msg_name = MSG_NAMES.get(frame.msg_id, f"MSG_{frame.msg_id}")
    out: dict[str, Any] = {
        "type": "um981_binary",
        "message": msg_name,
        "message_id": frame.msg_id,
        "body_len": len(frame.body),
    }
    try:
        if frame.msg_id == 1465:
            out["decoded"] = decode_binary_inspvax(frame.body)
        elif frame.msg_id == 1461:
            out["decoded"] = decode_binary_rawimux(frame.body)
        elif frame.msg_id == 1442:
            out["decoded"] = decode_binary_imuatt(frame.body)
        elif frame.msg_id in {1443, 1444}:
            out["decoded"] = decode_binary_gyratt(frame.body)
        elif frame.msg_id == 57024:
            out["decoded"] = decode_binary_drpva(frame.body)
    except struct.error as exc:
        out["decode_error"] = str(exc)
    return out


def decode_binary_inspvax(body: bytes) -> dict[str, Any]:
    values = struct.unpack_from("<IIddd f ddd ddd fffffffff I H", body, 0)
    return {
        "ins_status": INS_STATUS.get(values[0], values[0]),
        "pos_type": POS_TYPE.get(values[1], values[1]),
        "lat_deg": values[2],
        "lon_deg": values[3],
        "height_m": values[4],
        "undulation_m": values[5],
        "north_vel_mps": values[6],
        "east_vel_mps": values[7],
        "up_vel_mps": values[8],
        "roll_deg": values[9],
        "pitch_deg": values[10],
        "azimuth_deg": values[11],
        "lat_sigma_m": values[12],
        "lon_sigma_m": values[13],
        "height_sigma_m": values[14],
        "north_vel_sigma_mps": values[15],
        "east_vel_sigma_mps": values[16],
        "up_vel_sigma_mps": values[17],
        "roll_sigma_deg": values[18],
        "pitch_sigma_deg": values[19],
        "azimuth_sigma_deg": values[20],
        "ext_sol_stat": values[21],
        "time_since_update_s": values[22],
    }


def decode_binary_rawimux(body: bytes) -> dict[str, Any]:
    imu_error, imu_type, week, seconds, imu_status, za, nya, xa, zg, nyg, xg = struct.unpack_from(
        "<BBH d I llllll", body, 0
    )
    decoded: dict[str, Any] = {
        "imu_error": imu_error,
        "imu_type": imu_type,
        "week": week,
        "seconds": seconds,
        "imu_status": imu_status,
        "raw": [za, nya, xa, zg, nyg, xg],
    }
    if imu_type == 64:
        accel_scale = 2.0 * 9.80665 / 32767.0
        gyro_scale = 250.0 / 32767.0
        decoded.update(
            {
                "z_accel_mps2": za * accel_scale,
                "neg_y_accel_mps2": nya * accel_scale,
                "x_accel_mps2": xa * accel_scale,
                "z_gyro_dps": zg * gyro_scale,
                "neg_y_gyro_dps": nyg * gyro_scale,
                "x_gyro_dps": xg * gyro_scale,
            }
        )
    return decoded


def decode_binary_imuatt(body: bytes) -> dict[str, Any]:
    ins_status, pos_type, sol_age, dr_age, roll, pitch, azimuth, ax, ay, az, gx, gy, gz, _, _ = struct.unpack_from(
        "<IIIHhhhhhhhhhhh", body, 0
    )
    angle_scale = 360.0 / 32767.0
    accel_scale = 80.0 / 32767.0
    gyro_scale = 500.0 / 32767.0
    return {
        "ins_status": INS_STATUS.get(ins_status, ins_status),
        "pos_type": POS_TYPE.get(pos_type, pos_type),
        "sol_age_0p001s": sol_age,
        "dr_age_0p1s": dr_age,
        "roll_deg": roll * angle_scale,
        "pitch_deg": pitch * angle_scale,
        "azimuth_deg": azimuth * angle_scale,
        "x_accel_mps2": ax * accel_scale,
        "y_accel_mps2": ay * accel_scale,
        "z_accel_mps2": az * accel_scale,
        "x_gyro_dps": gx * gyro_scale,
        "y_gyro_dps": gy * gyro_scale,
        "z_gyro_dps": gz * gyro_scale,
        "raw": [ax, ay, az, gx, gy, gz],
    }


def decode_binary_gyratt(body: bytes) -> dict[str, Any]:
    ins_status, pos_type, dr_age, sol_age, roll, pitch, azimuth, gyro_z, _, _ = struct.unpack_from(
        "<BBH I hhhhhh", body, 0
    )
    angle_scale = 360.0 / 32767.0
    gyro_scale = 500.0 / 32767.0
    return {
        "ins_status": INS_STATUS.get(ins_status, ins_status),
        "pos_type": POS_TYPE.get(pos_type, pos_type),
        "dr_age_0p1s": dr_age,
        "sol_age_0p001s": sol_age,
        "roll_deg": roll * angle_scale,
        "pitch_deg": pitch * angle_scale,
        "azimuth_deg": azimuth * angle_scale,
        "gyro_z_dps": gyro_z * gyro_scale,
    }


def decode_binary_drpva(body: bytes) -> dict[str, Any]:
    fmt = "<III 4s ff ddd f fff ddd fff ddd fff 4l 4f 4d"
    values = struct.unpack_from(fmt, body, 0)
    return {
        "sol_status": SOL_STATUS.get(values[0], values[0]),
        "pos_type": POS_TYPE.get(values[1], values[1]),
        "datum_id": values[2],
        "dr_age_s": values[4],
        "sol_age_s": values[5],
        "lat_deg": values[6],
        "lon_deg": values[7],
        "height_m": values[8],
        "undulation_m": values[9],
        "lat_sigma_m": values[10],
        "lon_sigma_m": values[11],
        "height_sigma_m": values[12],
        "east_vel_mps": values[13],
        "north_vel_mps": values[14],
        "up_vel_mps": values[15],
        "heading_deg": values[19],
        "pitch_deg": values[20],
        "roll_deg": values[21],
        "heading_sigma_deg": values[22],
        "pitch_sigma_deg": values[23],
        "roll_sigma_deg": values[24],
    }


def extract_frames(buffer: bytearray) -> list[Frame]:
    frames: list[Frame] = []

    while buffer:
        first_positions = [pos for pos in [buffer.find(b"#"), buffer.find(b"%"), buffer.find(b"$")] if pos >= 0]
        bin_positions = [pos for pos in [buffer.find(SYNC_NORMAL), buffer.find(SYNC_SHORT)] if pos >= 0]
        candidates = first_positions + bin_positions
        if not candidates:
            if len(buffer) > 4096:
                frames.append(Frame(kind="raw_bytes", raw=bytes(buffer[:4096])))
                del buffer[:4096]
            break

        start = min(candidates)
        if start > 0:
            frames.append(Frame(kind="raw_bytes", raw=bytes(buffer[:start])))
            del buffer[:start]
            continue

        if buffer.startswith((b"#", b"%", b"$")):
            newline_positions = [pos for pos in [buffer.find(b"\n"), buffer.find(b"\r")] if pos >= 0]
            if not newline_positions:
                break
            end = min(newline_positions) + 1
            raw = bytes(buffer[:end])
            del buffer[:end]
            while buffer[:1] in (b"\r", b"\n"):
                del buffer[:1]
            frames.append(Frame(kind="ascii", raw=raw, text=clean_ascii(raw)))
            continue

        if buffer.startswith(SYNC_NORMAL):
            if len(buffer) < 28:
                break
            header_len = buffer[3]
            if header_len < 28:
                frames.append(Frame(kind="raw_bytes", raw=bytes(buffer[:1])))
                del buffer[:1]
                continue
            if len(buffer) < header_len:
                break
            msg_id = struct.unpack_from("<H", buffer, 4)[0]
            msg_len = struct.unpack_from("<H", buffer, 8)[0]
            total = header_len + msg_len + 4
            if len(buffer) < total:
                break
            raw = bytes(buffer[:total])
            del buffer[:total]
            frames.append(
                Frame(
                    kind="binary",
                    raw=raw,
                    msg_id=msg_id,
                    header_len=header_len,
                    body=raw[header_len : header_len + msg_len],
                )
            )
            continue

        if buffer.startswith(SYNC_SHORT):
            if len(buffer) < 12:
                break
            msg_len = buffer[3]
            msg_id = struct.unpack_from("<H", buffer, 4)[0]
            total = 12 + msg_len + 4
            if len(buffer) < total:
                break
            raw = bytes(buffer[:total])
            del buffer[:total]
            frames.append(Frame(kind="binary", raw=raw, msg_id=msg_id, header_len=12, body=raw[12 : 12 + msg_len]))
            continue

        frames.append(Frame(kind="raw_bytes", raw=bytes(buffer[:1])))
        del buffer[:1]

    return frames


def decode_frame(frame: Frame) -> dict[str, Any]:
    if frame.kind == "ascii":
        return parse_ascii_sentence(frame.text or "")
    if frame.kind == "binary":
        return decode_binary(frame)
    return {"type": frame.kind, "raw_hex": frame.raw.hex(" ")}
