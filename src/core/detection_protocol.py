import struct
import math


DETECTION_PAYLOAD_TYPE = 200
DETECTION_PROTOCOL_VERSION = 1
DETECTION_CLASS_NAME_SIZE = 32
DETECTION_STRUCT = struct.Struct("<BIIfffii32s")


def decode_detection_payload(payload):
    raw = bytes(payload)
    if len(raw) < DETECTION_STRUCT.size:
        raise ValueError("Detection payload is shorter than expected")

    (
        version,
        sequence,
        frame_id,
        confidence,
        depth,
        angle,
        latitude_e7,
        longitude_e7,
        class_bytes,
    ) = DETECTION_STRUCT.unpack(raw[:DETECTION_STRUCT.size])
    if version != DETECTION_PROTOCOL_VERSION:
        raise ValueError(f"Unsupported detection protocol version: {version}")
    if not all(math.isfinite(value) for value in (confidence, depth, angle)):
        raise ValueError("Detection payload contains a non-finite numeric value")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("Detection confidence must be between 0 and 1")
    if depth < 0.0 or depth > 10000.0:
        raise ValueError("Detection depth is outside the supported range")
    if angle < -180.0 or angle > 180.0:
        raise ValueError("Detection angle is outside -180..180 degrees")
    latitude = latitude_e7 / 1e7
    longitude = longitude_e7 / 1e7
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError("Detection coordinates are outside valid latitude/longitude ranges")

    return {
        "sequence": sequence,
        "frame_id": frame_id,
        "confidence": confidence,
        "depth": depth,
        "angle": angle,
        "lat": latitude,
        "lon": longitude,
        "class_name": class_bytes.split(b"\x00", 1)[0].decode("utf-8", errors="replace"),
    }
