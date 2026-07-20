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
FOLLOW_MIN_MM = 600
FOLLOW_MAX_MM = 4000
# Wider vertical band — logs showed empty_band on Y1–5 most of the walk.
PERSON_Y_MIN = 0
PERSON_Y_MAX = 6
SMOOTH_SAMPLES = 5
WARMUP_FRAMES = 8
# Hold last good range through brief torso-band / USB misses (~1.2 s at 10 Hz).
NO_READING_LOST_FRAMES = 12
# USB 8x8 stream is stale if no committed frame arrives within this window.
USB_FRAME_STALE_S = 0.45
# Person = nearest blob in the torso band. Cells within this window of the
# nearest valid cell count as the person; anything farther is background.
PERSON_BLOB_WINDOW_MM = 400
MIN_BLOB_CELLS = 2
# At follow distance an 8x8 often lights only one torso cell — allow it when far.
FAR_SINGLE_CELL_MM = 1500
# Reject single-frame flashes (floor/wall) that jump farther than this from hold.
MAX_FRAME_JUMP_CM = 55

# Absolute follow-distance state thresholds.
# Reverse protects personal space; forward catch-up has a separate band.
BACKUP_START_DISTANCE_CM = 220
BACKUP_STOP_DISTANCE_CM = 235
FORWARD_STOP_DISTANCE_CM = 240
FORWARD_START_DISTANCE_CM = 250
# While walking, catch-up authority rises with the gap and gets an extra boost
# past this soft maximum so the cart works to stay within about 3.5 m.
WALKING_MAX_GAP_CM = 350
MAX_THROTTLE = 512
# P corrects accumulated spacing error; D matches the user's range rate.
KP_THROTTLE = 2.0
KD_THROTTLE = 1.4
FAR_GAP_BOOST_KP = 2.0
MAX_RANGE_RATE_CMS = 140.0
RANGE_RATE_DEADZONE_CMS = 6.0
# Floor for any active correction. With follow_max_speed=110, raw 140 scales
# to ~30 motor units so catch-up/reverse starts decisively instead of creeping.
MIN_THROTTLE = 140
MIN_COMMAND_THROTTLE = 8
# After a correction settles into HOLD, don't start the opposite correction
# for this long — the cart's stopping overshoot (~10–25 cm) otherwise walks
# it across the narrow neutral band and back, oscillating ±0.25 m.
DIRECTION_FLIP_LOCKOUT_S = 1.5
# Personal space always wins: inside this distance backup starts regardless
# of the lockout.
BACKUP_OVERRIDE_DISTANCE_CM = 200

FOLLOW_LOST = "lost"
FOLLOW_HOLD = "hold"
FOLLOW_CATCH_UP = "catch_up"
FOLLOW_BACKUP = "backup"

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
        self.miss_reason = ""
        self.frame_age_s = 0.0
        self._prev_distance_cm = 0
        self._prev_distance_time = 0.0
        self._range_rate_cms = 0.0
        self.motion_state = FOLLOW_LOST
        self._last_correction = ""
        self._hold_since = 0.0
        self._last_error: Optional[str] = None
        self._grid_lock = threading.Lock()
        self._latest_buf = [0] * MATRIX_POINTS
        self._last_frame_time = 0.0
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
            self._last_frame_time = time.time()
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

    def _grid_snapshot(self) -> tuple[list[int], float]:
        """Return (grid, age_s). age_s is 0 for fresh I2C reads."""
        if self._mode == "i2c":
            if not self.connected or self._sensor is None:
                return [0] * MATRIX_POINTS, 999.0
            raw = self._sensor.get_all_data()
            if not raw:
                return [0] * MATRIX_POINTS, 999.0
            with self._grid_lock:
                self._last_frame_time = time.time()
            return self._decode_grid(raw), 0.0

        with self._grid_lock:
            age = (
                time.time() - self._last_frame_time
                if self._last_frame_time > 0
                else 999.0
            )
            return list(self._latest_buf), age

    def read_distance_cm(self) -> int:
        self.valid_count = 0
        self.grid_valid = 0
        self.min_mm = 0
        self.center_mm = [0, 0, 0, 0]
        self.reading_held = False
        self.miss_reason = ""
        self.frame_age_s = 0.0

        if not self.connected:
            self.distance_cm = 0
            self.throttle = 0
            self.miss_reason = "disconnected"
            self._reset_motion_control()
            return 0

        frame_cm, reason = self._measure_frame_cm()
        self.miss_reason = reason
        if frame_cm > 0:
            # Soft spike reject: one wild flash vs held range is treated as a miss.
            if (
                self._held_distance_cm > 0
                and abs(frame_cm - self._held_distance_cm) > MAX_FRAME_JUMP_CM
            ):
                self.miss_reason = "jump"
                frame_cm = 0

        if frame_cm > 0:
            self._miss_streak = 0
            self.miss_reason = ""
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
        self._reset_motion_control()
        return 0

    def _reset_motion_control(self) -> None:
        """Reset range-rate history and require a fresh state acquisition."""
        self._prev_distance_cm = 0
        self._prev_distance_time = 0.0
        self._range_rate_cms = 0.0
        self.motion_state = FOLLOW_LOST
        self._last_correction = ""
        self._hold_since = 0.0

    def _measure_frame_cm(self) -> tuple[int, str]:
        """Nearest torso-band blob distance, or (0, miss_reason).

        The person is the nearest coherent object in the band. Background
        cells behind them are excluded so a 2.2 m person is not read as a
        farther wall.
        """
        buf, age = self._grid_snapshot()
        self.frame_age_s = age
        if self._mode != "i2c" and age > USB_FRAME_STALE_S:
            return 0, "usb_stale"
        if not any(buf):
            return 0, "empty_grid"

        for i, (x, y) in enumerate(CENTER_POINTS):
            self.center_mm[i] = buf[y * MATRIX_SIZE + x]

        band_mm: list[int] = []
        for y in range(PERSON_Y_MIN, PERSON_Y_MAX + 1):
            for x in range(MATRIX_SIZE):
                distance_mm = buf[y * MATRIX_SIZE + x]
                if self._follow_valid(distance_mm):
                    band_mm.append(distance_mm)
        self.grid_valid = len(band_mm)

        if not band_mm:
            return 0, "empty_band"

        nearest = min(band_mm)
        blob = [d for d in band_mm if d - nearest <= PERSON_BLOB_WINDOW_MM]
        min_cells = 1 if nearest >= FAR_SINGLE_CELL_MM else MIN_BLOB_CELLS
        if len(blob) < min_cells:
            return 0, "blob_small"

        self.valid_count = len(blob)
        self.min_mm = nearest
        frame_cm = int(statistics.median(blob) / 10)
        if frame_cm <= 0:
            return 0, "empty_band"
        return frame_cm, ""

    def _flip_locked(self, wanted_state: str, now: float) -> bool:
        """True while the opposite correction is still in its settle lockout.

        Stopping overshoot (~10–25 cm) can carry the cart across the neutral
        band right after a correction ends; without this it ping-pongs
        backup ↔ catch_up about ±0.25 m.
        """
        if self._last_correction in ("", wanted_state):
            return False
        return (now - self._hold_since) < DIRECTION_FLIP_LOCKOUT_S

    def _update_motion_state(self, distance_cm: float) -> None:
        """Select one stable motion state from absolute distance thresholds."""
        now = time.time()
        previous_state = self.motion_state
        if self.motion_state == FOLLOW_LOST:
            if distance_cm > FORWARD_START_DISTANCE_CM:
                self.motion_state = FOLLOW_CATCH_UP
            elif distance_cm <= BACKUP_START_DISTANCE_CM:
                self.motion_state = FOLLOW_BACKUP
            else:
                self.motion_state = FOLLOW_HOLD
        elif self.motion_state == FOLLOW_HOLD:
            if distance_cm > FORWARD_START_DISTANCE_CM and not self._flip_locked(
                FOLLOW_CATCH_UP, now
            ):
                self.motion_state = FOLLOW_CATCH_UP
            elif distance_cm <= BACKUP_START_DISTANCE_CM and (
                distance_cm <= BACKUP_OVERRIDE_DISTANCE_CM
                or not self._flip_locked(FOLLOW_BACKUP, now)
            ):
                self.motion_state = FOLLOW_BACKUP
        elif self.motion_state == FOLLOW_CATCH_UP:
            if distance_cm <= BACKUP_START_DISTANCE_CM:
                self.motion_state = FOLLOW_BACKUP
            elif distance_cm <= FORWARD_STOP_DISTANCE_CM:
                # Keep chasing while the gap is still growing (user walking away).
                # Stopping here caused stop-go at ~2.4 m during a normal walk.
                if self._range_rate_cms <= RANGE_RATE_DEADZONE_CMS:
                    self.motion_state = FOLLOW_HOLD
        elif self.motion_state == FOLLOW_BACKUP:
            if distance_cm >= BACKUP_STOP_DISTANCE_CM:
                self.motion_state = FOLLOW_HOLD
        else:
            self.motion_state = FOLLOW_LOST

        # Do not carry catch-up/backup momentum through HOLD and turn it into
        # an immediate opposite command. Motor deceleration handles the stop;
        # fresh range-rate samples can then match new user movement.
        if (
            self.motion_state == FOLLOW_HOLD
            and previous_state in (FOLLOW_CATCH_UP, FOLLOW_BACKUP)
        ):
            self._range_rate_cms = 0.0
            self._last_correction = previous_state
            self._hold_since = now

    def _throttle_from_distance(self, distance_cm: int) -> int:
        """Stateful PD: stable spacing correction plus walking-speed matching."""
        now = time.time()
        if distance_cm <= 0:
            self._reset_motion_control()
            return 0

        if self._prev_distance_cm > 0 and self._prev_distance_time > 0:
            dt = max(0.05, min(0.5, now - self._prev_distance_time))
            raw_rate = (distance_cm - self._prev_distance_cm) / dt
            raw_rate = max(-MAX_RANGE_RATE_CMS, min(MAX_RANGE_RATE_CMS, raw_rate))
            self._range_rate_cms = 0.35 * raw_rate + 0.65 * self._range_rate_cms
        self._prev_distance_cm = distance_cm
        self._prev_distance_time = now

        self._update_motion_state(float(distance_cm))

        # P is measured from each correction's stop threshold. This creates
        # hysteresis without driving while inside the neutral 2.35–2.5 m band.
        if self.motion_state == FOLLOW_CATCH_UP:
            p_error = max(0.0, distance_cm - FORWARD_STOP_DISTANCE_CM)
        elif self.motion_state == FOLLOW_BACKUP:
            p_error = min(0.0, distance_cm - BACKUP_STOP_DISTANCE_CM)
        else:
            p_error = 0.0

        # Ignore tiny range-rate noise. D becomes active after catch-up/backup
        # starts, allowing the cart to match walking speed during that motion.
        if abs(self._range_rate_cms) <= RANGE_RATE_DEADZONE_CMS:
            range_rate = 0.0
        else:
            range_rate = self._range_rate_cms - (
                RANGE_RATE_DEADZONE_CMS
                if self._range_rate_cms > 0
                else -RANGE_RATE_DEADZONE_CMS
            )

        if self.motion_state == FOLLOW_HOLD:
            # Respect the requested absolute start thresholds. Once a correction
            # starts, D remains active to match the user's walking speed.
            throttle = 0
        else:
            throttle = int(KP_THROTTLE * p_error + KD_THROTTLE * range_rate)
            if (
                self.motion_state == FOLLOW_CATCH_UP
                and distance_cm > WALKING_MAX_GAP_CM
            ):
                throttle += int(
                    FAR_GAP_BOOST_KP * (distance_cm - WALKING_MAX_GAP_CM)
                )

        # A correction state cannot command the opposite direction. It can
        # reduce to zero and let the strong motor deceleration stop the cart.
        if self.motion_state == FOLLOW_CATCH_UP:
            throttle = max(0, throttle)
        elif self.motion_state == FOLLOW_BACKUP:
            throttle = min(0, throttle)

        if abs(throttle) < MIN_COMMAND_THROTTLE:
            return 0
        if abs(throttle) < MIN_THROTTLE:
            throttle = MIN_THROTTLE if throttle > 0 else -MIN_THROTTLE
        return max(-MAX_THROTTLE, min(MAX_THROTTLE, throttle))

    def read_loop(self, running_cb, interval_s: float = 0.1, log_cb=None):
        """Background loop: call running_cb() until it returns False."""
        while running_cb():
            cm = self.read_distance_cm()
            if log_cb:
                source = (
                    f"usb={self.port}"
                    if self._mode != "i2c"
                    else f"addr=0x{self._addr:02X}"
                )
                if cm > 0:
                    held = " sample-held" if self.reading_held else ""
                    miss = f" miss={self.miss_reason}" if self.miss_reason else ""
                    log_cb(
                        f"LiDAR cm={cm} throttle={self.throttle} "
                        f"state={self.motion_state} rate={self._range_rate_cms:.0f}cm/s "
                        f"valid={self.valid_count} grid={self.grid_valid} "
                        f"min_mm={self.min_mm} age={self.frame_age_s:.2f}s"
                        f"{held}{miss} {source}"
                    )
                elif self.miss_reason:
                    log_cb(
                        f"LiDAR no-reading miss={self.miss_reason} "
                        f"age={self.frame_age_s:.2f}s streak={self._miss_streak} "
                        f"{source}"
                    )
            time.sleep(interval_s)
