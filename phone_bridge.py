"""TCP bridge: Android phone sends follow-me steering packets over USB tethering."""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional

from cart_protocol import START_BYTE, extract_packets, parse_packet

DEFAULT_PORT = 9747
DEFAULT_HOST = "0.0.0.0"


class PhoneBridge:
    """
    Listen for framed cart packets from the phone app.

    Phone vision provides steering; Pi LiDAR provides throttle. The phone
    throttle field is used as backup when LiDAR has no reading.

    ASCII status lines (CALIB_START, CALIB 3/2/1, CALIB_OK, CALIB_FAIL) may be
    interleaved with binary packets on the same TCP stream.
    """

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        log_cb: Optional[Callable[[str], None]] = None,
        status_cb: Optional[Callable[[str], None]] = None,
    ):
        self.host = host
        self.port = port
        self._log_cb = log_cb
        self._status_cb = status_cb
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._client_gen = 0

        self.connected = False
        self.last_packet_time = 0.0
        self.steering = 0
        self.phone_throttle = 0
        self.person_detected = False
        self.calibrating = False
        self.calib_ok = False
        self.packets_received = 0
        self.last_track_line = ""
        self.last_client_addr = ""
        self._last_logged_steering = None
        self._last_logged_throttle = None
        self._last_logged_detected = None
        self._last_logged_calibrating = None
        self._last_logged_calib_ok = None
        self._last_client_ip = ""

    def _log(self, message: str):
        if self._log_cb:
            self._log_cb(message)

    def _client_ip(self, addr) -> str:
        return str(addr[0])

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._serve_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        for sock in (self._client_sock, self._server_sock):
            if sock:
                try:
                    sock.close()
                except OSError:
                    pass
        self._client_sock = None
        self._server_sock = None
        self.connected = False

    def is_stale(self, timeout_s: float = 2.0) -> bool:
        if not self.connected:
            return True
        if self.last_packet_time == 0.0:
            return False
        return (time.time() - self.last_packet_time) > timeout_s

    def _serve_loop(self):
        while self._running:
            try:
                self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._server_sock.settimeout(1.0)
                self._server_sock.bind((self.host, self.port))
                self._server_sock.listen(5)
                self._log(f"Phone bridge listening on {self.host}:{self.port}")
                while self._running:
                    try:
                        client, addr = self._server_sock.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    self._start_client(client, addr)
            except OSError as e:
                if self._running:
                    self._log(f"Phone bridge socket error: {e} (retrying)")
                    time.sleep(1.0)
            finally:
                if self._server_sock:
                    try:
                        self._server_sock.close()
                    except OSError:
                        pass
                    self._server_sock = None

    def _start_client(self, client: socket.socket, addr):
        try:
            client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
        client.settimeout(30.0)

        client_ip = self._client_ip(addr)
        with self._lock:
            old = self._client_sock
            old_ip = self._last_client_ip
            recently_active = (
                old is not None
                and old_ip == client_ip
                and self.last_packet_time > 0
                and (time.time() - self.last_packet_time) < 5.0
            )
            if recently_active:
                try:
                    client.close()
                except OSError:
                    pass
                return

            self._client_gen += 1
            generation = self._client_gen
            self._client_sock = client
            self.connected = True
            self.last_client_addr = f"{addr[0]}:{addr[1]}"
            self._last_client_ip = client_ip
            self.calibrating = False
            self.calib_ok = False

        if old is not None and old is not client:
            try:
                old.close()
            except OSError:
                pass
        self._log(f"Phone connected from {addr[0]}:{addr[1]}")
        threading.Thread(
            target=self._handle_client,
            args=(client, addr, generation),
            daemon=True,
        ).start()

    def _handle_client(self, client: socket.socket, addr, generation: int):
        buffer = bytearray()
        try:
            while self._running:
                with self._lock:
                    superseded = generation != self._client_gen
                if superseded:
                    break
                try:
                    chunk = client.recv(512)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buffer.extend(chunk)
                buffer = self._drain_ascii_lines(buffer)
                packets, remainder = extract_packets(buffer)
                buffer = bytearray(remainder)
                for packet in packets:
                    self._apply_packet(packet, addr)
        finally:
            try:
                client.close()
            except OSError:
                pass
            with self._lock:
                still_current = self._client_sock is client
                if still_current:
                    self._client_sock = None
                    self.connected = False
                    self.steering = 0
                    self.phone_throttle = 0
                    self.person_detected = False
                    self.calibrating = False
            if still_current:
                self._log(f"Phone disconnected ({addr[0]}:{addr[1]})")

    def _drain_ascii_lines(self, buffer: bytearray) -> bytearray:
        while buffer and buffer[0] != START_BYTE:
            newline_idx = buffer.find(b"\n")
            if newline_idx == -1:
                if len(buffer) > 128:
                    del buffer[0]
                break
            line = buffer[:newline_idx].decode("ascii", errors="ignore").strip()
            del buffer[: newline_idx + 1]
            if line:
                self._handle_status_line(line)
        return buffer

    def _handle_status_line(self, line: str):
        if line.startswith("CALIB"):
            if line == "CALIB_START":
                with self._lock:
                    self.calibrating = True
                    self.calib_ok = False
                self._log("🎨 Body calibration starting — stand centered in camera view")
            elif line.startswith("CALIB_PREP "):
                n = line[11:].strip()
                self._log(f"🎨 Get in position: {n}")
            elif line.startswith("CALIB_PHASE "):
                phase = line[12:].strip()
                self._log(f"🎨 Calibrating ({phase})")
            elif line.startswith("CALIB ") and line[6:].strip().isdigit():
                n = line[6:].strip()
                self._log(f"🎨 Calibration countdown: {n}")
            elif line.startswith("CALIB_OK"):
                with self._lock:
                    self.calibrating = False
                    self.calib_ok = True
                if self._status_cb:
                    self._status_cb(line)
                else:
                    self._log(f"✓ Body calibration OK ({line[9:].strip() or 'complete'})")
            elif line == "CALIB_FAIL":
                with self._lock:
                    self.calibrating = False
                    self.calib_ok = False
                self._log("✗ Body calibration failed — reconnect and try again")
            else:
                self._log(line)
        elif self._status_cb:
            self._status_cb(line)

    def _apply_packet(self, packet: bytes, addr):
        parsed = parse_packet(packet)
        if parsed is None:
            return

        steering = parsed["steering"]
        throttle = parsed["throttle"]
        buttons = parsed["button_states"]
        detected = bool(buttons & 0x80)
        calibrating = bool(buttons & 0x40)
        calib_ok = bool(buttons & 0x20)

        if calibrating:
            steering = 0
            throttle = 0

        with self._lock:
            self.steering = steering
            self.phone_throttle = throttle
            self.person_detected = detected
            self.calibrating = calibrating
            self.calib_ok = calib_ok
            self.last_packet_time = time.time()
            self.packets_received += 1
            self.last_track_line = (
                f"phone {addr[0]} s={steering} t={throttle} det={detected}"
            )

        if self._log_cb:
            changed = (
                steering != self._last_logged_steering
                or throttle != self._last_logged_throttle
                or detected != self._last_logged_detected
                or calibrating != self._last_logged_calibrating
                or calib_ok != self._last_logged_calib_ok
            )
            if not changed and self.packets_received % 50 != 0:
                return
            self._last_logged_steering = steering
            self._last_logged_throttle = throttle
            self._last_logged_detected = detected
            self._last_logged_calibrating = calibrating
            self._last_logged_calib_ok = calib_ok
            cal_txt = " cal=on" if calibrating else (" cal=ok" if calib_ok else "")
            self._log_cb(
                f"phone s={steering} t={throttle} det={detected}{cal_txt} "
                f"(#{self.packets_received})"
            )

    def get_steering(self) -> int:
        with self._lock:
            if self.is_stale() or self.calibrating:
                return 0
            return self.steering

    def get_throttle(self) -> int:
        with self._lock:
            if self.is_stale() or self.calibrating:
                return 0
            return self.phone_throttle

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "connected": self.connected,
                "steering": self.steering,
                "phone_throttle": self.phone_throttle,
                "person_detected": self.person_detected,
                "calibrating": self.calibrating,
                "calib_ok": self.calib_ok,
                "stale": self.is_stale(),
                "packets": self.packets_received,
                "track": self.last_track_line,
                "client": self.last_client_addr,
            }
