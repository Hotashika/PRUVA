import math
import struct


DECISION_PAYLOAD_TYPE = 201
DECISION_PROTOCOL_VERSION = 1
DECISION_STRUCT = struct.Struct("<BIBbHHf20s30s44s16s")
MISSION_ID_TO_NAME = {1: "task1", 2: "task2", 3: "task3", 4: "task4"}


def decode_decision_payload(payload):
    raw = bytes(payload)
    if len(raw) < DECISION_STRUCT.size:
        raise ValueError("Decision payload is shorter than expected")
    (
        version,
        sequence,
        mission_id,
        risk_value,
        current_target,
        target_count,
        progress_percent,
        stage,
        action,
        reason,
        colreg_rule,
    ) = DECISION_STRUCT.unpack(raw[:DECISION_STRUCT.size])
    if version != DECISION_PROTOCOL_VERSION:
        raise ValueError(f"Unsupported decision protocol version: {version}")
    if risk_value not in (-1, 0, 1):
        raise ValueError("Invalid collision-risk value")
    if not math.isfinite(progress_percent) or not 0.0 <= progress_percent <= 100.0:
        raise ValueError("Invalid mission progress value")

    def text(raw_value):
        return raw_value.split(b"\x00", 1)[0].decode("utf-8", errors="replace")

    return {
        "sequence": sequence,
        "active_mission": MISSION_ID_TO_NAME.get(mission_id, "unknown"),
        "stage": text(stage),
        "action": text(action),
        "reason": text(reason),
        "colreg_rule": text(colreg_rule),
        "collision_risk": None if risk_value == -1 else bool(risk_value),
        "current_target": current_target,
        "target_count": target_count,
        "progress_percent": progress_percent,
    }
