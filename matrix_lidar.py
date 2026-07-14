"""DFRobot SEN0628 Matrix LiDAR follow-distance reader for Raspberry Pi.

Default: USB-C into the Pi hub (streams 8x8 text at 115200 baud).
Fallback: I2C via ``MATRIX_LIDAR_MODE=i2c`` (GPIO2/3, DIP = I2C).
"""

from __future__ import annotations

import os
import re
import statistics
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import serial
import serial.tools.list_ports

LIB_DIR = Path(__file__).resolve().parent / "lib"
if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))

from DFRobot_matrixLidar import DFRobot_matrixLidar_i2c  # noqa: E402

# Sensor / grid
MATRIX_SIZE = 8
MATRIX_POINTS = 64
INVALID_MM = 4000
MIN_MM = 20
MAX_MM = 4000
I2C_ADDRESSES = (0x33, 0x32, 0x31, 0x30)
BOOT_DELAY_S = 2.5
LIDAR_BAUD = 115200
LIDAR_SYMLINK = "/dev/matrix-lidar"
RP2040_VID = "2E8A"
USB_FRAME_TIMEOUT_S = 3.0

# Follow-me LiDAR filtering (person blob + stall logic)
FOLLOW_MIN_MM = 300
FOLLOW_MAX_MM = 3500
PERSON_Y_MIN = 1
PERSON_Y_MAX = 5
FLOOR_Y_MIN = 6
MIN_VALID_CELLS = 3
SMOOTH_SAMPLES = 5
WARMUP_FRAMES = 8
NO_READING_LOST_FRAMES = 3  # hold last distance until this many misses in a row

# Throttle mapping
TARGET_DISTANCE_CM = 200
DISTANCE_DEADZONE_CM = 15
MAX_THROTTLE = 400
MIN_THROTTLE = 50

CENTER_POINTS = ((3, 3), (3, 4), (4, 3), (4, 4))
_Y_LINE_RE = re.compile(r"^Y(\d):\s*(.+)$", re.IGNORECASE)


def _is_xiao_camera_port(port) -> bool:
    desc = (getattr(port, "description", None) or "").lower()
    hwid = (getattr(port, "hwid", None) or "").upper()
    if any(identifier in desc for identifier in ("xiao", "seeed")):
        return True
    if "303A:1001" in hwid or "VID:PID=303A" in hwid:
        return True
    return "jtag" in desc and "serial" in desc


def _is_receiver_port(port) -> bool:
    desc = (getattr(port, "description", None) or "").lower()
    return any(
        identifier in desc
        for identifier in ("esp32", "cp2102", "cp210x", "ch340", "usb serial")
    )


def find_matrix_lidar_port() -> Optional[str]:
    """Auto-detect SEN0628 USB-C serial (RP2040 CDC), excluding ESP32 ports."""
    if os.path.exists(LIDAR_SYMLINK):
        return LIDAR_SYMLINK

    env_port = os.environ.get("MATRIX_LIDAR_PORT")
    if env_port:
        return env_port

    ports = list(serial.tools.list_ports.comports())
    candidates: list[tuple[int, str]] = []

    for port in ports:
        if _is_xiao_camera_port(port) or _is_receiver_port(port):
            continue

        desc = (port.description or "").lower()
        hwid = (port.hwid or "").upper()
        score = 0

        if RP2040_VID in hwid:
            score += 10
        if any(token in desc for token in ("rp2040", "pico", "board cdc", "matrix", "dfrobot")):
            score += 8
        if port.device.startswith("/dev/ttyACM"):
            score += 2
        if score > 0:
            candidates.append((score, port.device))

    if candidates:
        candidates.sort(key=lambda item: (-item[0], item[1]))
        return candidates[0][1]

    for port in ports:
        if _is_xiao_camera_port(port) or _is_receiver_port(port):
            continue
        if port.device.startswith("/dev/ttyACM"):
            return port.device

    return None


class MatrixLidarFollow:
    """Read 8x8 ToF grid and produce follow distance + throttle."""

    def __init__(self, port: Optional[str] = None, mode: Optional[str] = None):
        self._port = port
        self._mode = (mode or os.environ.get("MATRIX_LIDAR_MODE", "usb")).lower()
        self._bus_num = 1
        self._sensor = None
        self._serial: Optional[serial.Serial] = None
        self._addr = 0
        self.connected = False
        self.distance_cm = 0
        self.throttle = 0
        self.valid_count = 0
        self.grid_valid = 0
        self.min_mm = 0
        self.center_mm = [0, 0, 0, 0]
        self._smooth_buf: list[int] = []
        self._miss_streak = 0
        self._held_distance_cm = 0
        self._held_throttle = 0
        self.reading_held = False
        self._last_error: Optional[str] = None
        self._grid_lock = threading.Lock()
        self._latest_buf = [0] * MATRIX_POINTS
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_running = False
        self._first_frame = threading.Event()

    @property
    def port(self) -> str:
        if self._serial is not None:
            return self._serial.port or ""
        return self._port or ""

    @property
    def i2c_addr(self) -> int:
        return self._addr

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def connect(self) -> bool:
        if self._mode == "i2c":
            return self._connect_i2c()
        return self._connect_usb()

    def _connect_i2c(self) -> bool:
        time.sleep(BOOT_DELAY_S)
        for addr in I2C_ADDRESSES:
            candidate = DFRobot_matrixLidar_i2c(addr)
            for _ in range(5):
                if candidate.begin() == 0 and candidate.set_Ranging_Mode(8) == 0:
                    self._sensor = candidate
                    self._addr = addr
                    self.connected = True
                    self._last_error = None
                    for _ in range(WARMUP_FRAMES):
                        self.read_distance_cm()
                        time.sleep(0.05)
                    return True
                time.sleep(0.2)
        self.connected = False
        self._last_error = "Matrix LiDAR not found on I2C"
        return False

    def _connect_usb(self) -> bool:
        port = self._port or find_matrix_lidar_port()
        if not port:
            self.connected = False
            self._last_error = (
                "Matrix LiDAR USB port not found. Plug USB-C into the Pi hub, "
                f"or set MATRIX_LIDAR_PORT / symlink {LIDAR_SYMLINK}"
            )
            return False

        try:
            self._serial = serial.Serial(
                port,
                LIDAR_BAUD,
                timeout=0.2,
                write_timeout=0.2,
            )
            self._serial.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            self.connected = False
            self._last_error = f"Cannot open LiDAR USB port {port}: {exc}"
            return False

        self._port = port
        self._reader_running = True
        self._first_frame.clear()
        self._reader_thread = threading.Thread(
            target=self._usb_reader_loop,
            name="matrix-lidar-usb",
            daemon=True,
        )
        self._reader_thread.start()

        if not self._first_frame.wait(USB_FRAME_TIMEOUT_S):
            self._reader_running = False
            self._close_serial()
            self.connected = False
            self._last_error = (
                f"No 8x8 frames on {port} — is the LiDAR powered and USB-C connected? "
                "(Firmware streams Y0..Y7 lines at 115200.)"
            )
            return False

        self.connected = True
        self._last_error = None
        for _ in range(WARMUP_FRAMES):
            self.read_distance_cm()
            time.sleep(0.05)
        return True

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None

    def _commit_usb_frame(self, pending_rows: dict[int, list[int]]) -> None:
        if len(pending_rows) < MATRIX_SIZE:
            return
        buf = [0] * MATRIX_POINTS
        for y in range(MATRIX_SIZE):
            row = pending_rows.get(y, [0] * MATRIX_SIZE)
            for x in range(MATRIX_SIZE):
                buf[y * MATRIX_SIZE + x] = row[x] if x < len(row) else 0
        with self._grid_lock:
            self._latest_buf = buf
        self._first_frame.set()

    def _usb_reader_loop(self) -> None:
        pending_rows: dict[int, list[int]] = {}
        ser = self._serial
        if ser is None:
            return

        while self._reader_running:
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError):
                break
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("---"):
                self._commit_usb_frame(pending_rows)
                pending_rows.clear()
                continue

            match = _Y_LINE_RE.match(line)
            if not match:
                continue

            y = int(match.group(1))
            if y < 0 or y >= MATRIX_SIZE:
                continue

            values: list[int] = []
            for token in match.group(2).split(","):
                token = token.strip()
                if not token:
                    continue
                try:
                    values.append(int(token))
                except ValueError:
                    continue
            if not values:
                continue

            # New frame starting — commit the previous one (USB firmware has no --- line).
            if y == 0 and pending_rows:
                self._commit_usb_frame(pending_rows)
                pending_rows.clear()

            pending_rows[y] = values
            if y == MATRIX_SIZE - 1:
                self._commit_usb_frame(pending_rows)
                pending_rows.clear()

        self._reader_running = False

    @staticmethod
    def _follow_valid(distance_mm: int) -> bool:
        if distance_mm == 0 or distance_mm >= INVALID_MM:
            return False
        if distance_mm < MIN_MM or distance_mm > MAX_MM:
            return False
        return FOLLOW_MIN_MM <= distance_mm <= FOLLOW_MAX_MM

    def _decode_grid(self, raw: list[int]) -> list[int]:
        buf = []
        for i in range(0, len(raw), 2):
            if i + 1 >= len(raw):
                break
            buf.append((raw[i + 1] << 8) | raw[i])
        if len(buf) < MATRIX_POINTS:
            buf.extend([0] * (MATRIX_POINTS - len(buf)))
        return buf[:MATRIX_POINTS]

    def _grid_snapshot(self) -> list[int]:
        if self._mode == "i2c":
            if not self.connected or self._sensor is None:
                return [0] * MATRIX_POINTS
            raw = self._sensor.get_all_data()
            if not raw:
                return [0] * MATRIX_POINTS
            return self._decode_grid(raw)

        with self._grid_lock:
            return list(self._latest_buf)

    def read_distance_cm(self) -> int:
        self.valid_count = 0
        self.grid_valid = 0
        self.min_mm = 0
        self.center_mm = [0, 0, 0, 0]
        self.reading_held = False

        if not self.connected:
            self.distance_cm = 0
            self.throttle = 0
            return 0

        frame_cm = self._measure_frame_cm()
        if frame_cm > 0:
            self._miss_streak = 0
            self._smooth_buf.append(frame_cm)
            if len(self._smooth_buf) > SMOOTH_SAMPLES:
                self._smooth_buf.pop(0)
            smooth_cm = int(statistics.median(self._smooth_buf))
            self.distance_cm = smooth_cm
            self.throttle = self._throttle_from_distance(smooth_cm)
            self._held_distance_cm = smooth_cm
            self._held_throttle = self.throttle
            return smooth_cm

        self._miss_streak += 1
        if (
            self._miss_streak < NO_READING_LOST_FRAMES
            and self._held_distance_cm > 0
        ):
            self.distance_cm = self._held_distance_cm
            self.throttle = self._held_throttle
            self.reading_held = True
            return self._held_distance_cm

        self.distance_cm = 0
        self.throttle = 0
        return 0

    def _measure_frame_cm(self) -> int:
        """Raw per-frame distance (0 = no valid torso band this tick)."""
        buf = self._grid_snapshot()
        if not any(buf):
            return 0

        for i, (x, y) in enumerate(CENTER_POINTS):
            self.center_mm[i] = buf[y * MATRIX_SIZE + x]

        band_mm: list[int] = []
        center_mm: list[int] = []
        floor_count = 0

        for y in range(MATRIX_SIZE):
            for x in range(MATRIX_SIZE):
                distance_mm = buf[y * MATRIX_SIZE + x]
                if not self._follow_valid(distance_mm):
                    continue
                self.grid_valid += 1
                if y >= FLOOR_Y_MIN:
                    floor_count += 1
                if PERSON_Y_MIN <= y <= PERSON_Y_MAX:
                    band_mm.append(distance_mm)
                if (x, y) in CENTER_POINTS:
                    center_mm.append(distance_mm)

        if len(center_mm) >= 2:
            use_mm = center_mm
        elif len(band_mm) >= MIN_VALID_CELLS:
            use_mm = band_mm
        elif band_mm and floor_count == 0:
            use_mm = band_mm
        else:
            return 0

        if not use_mm:
            return 0

        self.valid_count = len(use_mm)
        self.min_mm = min(use_mm)
        frame_cm = int(statistics.median(use_mm) / 10)
        return frame_cm if frame_cm > 0 else 0

    @staticmethod
    def _throttle_from_distance(distance_cm: int) -> int:
        if distance_cm <= 0:
            return 0

        error_cm = distance_cm - TARGET_DISTANCE_CM
        if abs(error_cm) <= DISTANCE_DEADZONE_CM:
            return MIN_THROTTLE

        ratio = error_cm / TARGET_DISTANCE_CM
        throttle = int(ratio * MAX_THROTTLE)
        if 0 < abs(throttle) < MIN_THROTTLE:
            throttle = MIN_THROTTLE if throttle > 0 else -MIN_THROTTLE
        return max(-MAX_THROTTLE, min(MAX_THROTTLE, throttle))

    def read_loop(self, running_cb, interval_s: float = 0.1, log_cb=None):
        """Background loop: call running_cb() until it returns False."""
        while running_cb():
            cm = self.read_distance_cm()
            if log_cb and cm > 0:
                source = f"usb={self.port}" if self._mode != "i2c" else f"addr=0x{self._addr:02X}"
                log_cb(
                    f"LiDAR cm={cm} throttle={self.throttle} "
                    f"valid={self.valid_count} grid={self.grid_valid} "
                    f"min_mm={self.min_mm} {source}"
                )
            time.sleep(interval_s)
