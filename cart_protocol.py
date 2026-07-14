"""Binary follow-me packet protocol shared by the Android phone and the Pi."""

from __future__ import annotations

import struct
from typing import List, Optional, Tuple

START_BYTE = 0xAA
END_BYTE = 0xBB
PAYLOAD_SIZE = 13

# struct usb_data_t — little-endian, packed (matches hoverboard_minimal / ESP32)
_PAYLOAD_FMT = "<hhBBhBBBB"


def checksum(payload: bytes) -> int:
    return sum(payload[:12]) & 0xFF


def build_packet(
    throttle: int = 0,
    steering: int = 0,
    emergency_stop: int = 0,
    cruise_control: int = 0,
    cruise_speed: int = 0,
    follow_me_mode: int = 1,
    turbo_mode: int = 0,
    battery_level: int = 0,
    button_states: int = 0,
) -> bytes:
    body = struct.pack(
        _PAYLOAD_FMT,
        int(throttle),
        int(steering),
        int(emergency_stop) & 0xFF,
        int(cruise_control) & 0xFF,
        int(cruise_speed),
        int(follow_me_mode) & 0xFF,
        int(turbo_mode) & 0xFF,
        int(battery_level) & 0xFF,
        int(button_states) & 0xFF,
    )
    body += bytes([checksum(body)])
    return bytes([START_BYTE]) + body + bytes([END_BYTE])


def parse_packet(data: bytes) -> Optional[dict]:
    """Validate framing and return decoded fields, or None if invalid."""
    if len(data) < 15 or data[0] != START_BYTE or data[-1] != END_BYTE:
        return None

    payload = data[1:-1]
    if len(payload) != PAYLOAD_SIZE:
        return None

    fields = struct.unpack(_PAYLOAD_FMT, payload[:12])
    if (sum(payload[:12]) & 0xFF) != payload[12]:
        return None

    return {
        "throttle": fields[0],
        "steering": fields[1],
        "emergency_stop": fields[2],
        "cruise_control": fields[3],
        "cruise_speed": fields[4],
        "follow_me_mode": fields[5],
        "turbo_mode": fields[6],
        "battery_level": fields[7],
        "button_states": fields[8],
        "checksum": payload[12],
    }


def extract_packets(buffer: bytearray) -> Tuple[List[bytes], bytearray]:
    """Pull complete framed packets from a growing byte buffer."""
    packets = []
    while True:
        start = buffer.find(bytes([START_BYTE]))
        if start == -1:
            return packets, bytearray()
        if start > 0:
            del buffer[:start]
        end = buffer.find(bytes([END_BYTE]), 1)
        if end == -1:
            return packets, buffer
        packet = bytes(buffer[: end + 1])
        del buffer[: end + 1]
        packets.append(packet)
