"""Phone GPS fixes for golf yardages (replaces a Pi USB GPS dongle)."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Callable, Optional

try:
    from golf_green_yardages import load_course_data, yardages_for_hole
except ImportError:
    load_course_data = None  # type: ignore
    yardages_for_hole = None  # type: ignore


class PhoneGps:
    """
    Latest GPS fix from the Android app (ASCII ``GPS lat=… lon=…`` lines).

  Optional cached OSM course JSON (same format as ``golf_green_yardages.py``)
    for front / middle / back yardage logging on the Pi.
    """

    def __init__(
        self,
        course_cache: Optional[str] = None,
        default_hole: int = 1,
    ):
        self.lat = 0.0
        self.lon = 0.0
        self.accuracy_m = 0.0
        self.speed_mps = 0.0
        self.course_bearing_deg: Optional[float] = None
        self.compass_azimuth_deg: Optional[float] = None
        self.heading_update = 0.0
        self.last_update = 0.0
        self.hole = default_hole
        self.course_cache_path = course_cache
        self._course_data: Optional[dict[str, Any]] = None
        self._last_yardage_log = 0.0
        self._last_yardage_key: Optional[tuple] = None
        self._load_error = ""

        if course_cache:
            self._load_course(course_cache)

    def _load_course(self, path: str) -> None:
        self._load_error = ""
        if load_course_data is None:
            self._load_error = "golf_green_yardages not importable (pip install shapely)"
            return
        try:
            self._course_data = load_course_data(Path(path))
        except Exception as e:
            self._load_error = str(e)
            self._course_data = None

    def parse_line(self, line: str) -> bool:
        if line.startswith("GPS "):
            return self._parse_gps(line[4:])
        if line.startswith("HOLE "):
            return self._parse_hole(line[5:])
        if line.startswith("HEADING "):
            return self._parse_heading(line[8:])
        return False

    def _parse_gps(self, body: str) -> bool:
        fields = self._parse_kv(body)
        lat = fields.get("lat")
        lon = fields.get("lon")
        if lat is None or lon is None:
            return False
        self.lat = lat
        self.lon = lon
        self.accuracy_m = fields.get("acc", 0.0)
        self.speed_mps = fields.get("spd", 0.0)
        self.last_update = time.time()
        return True

    def _parse_heading(self, body: str) -> bool:
        fields = self._parse_kv(body)
        brg = fields.get("brg")
        if brg is None:
            return False
        self.course_bearing_deg = brg
        self.compass_azimuth_deg = fields.get("az")
        if "spd" in fields:
            self.speed_mps = fields["spd"]
        self.heading_update = time.time()
        return True

    def _parse_hole(self, body: str) -> bool:
        body = body.strip()
        if not body.isdigit():
            return False
        hole = int(body)
        if 1 <= hole <= 18:
            self.hole = hole
            self._last_yardage_key = None
            return True
        return False

    @staticmethod
    def _parse_kv(body: str) -> dict[str, float]:
        out: dict[str, float] = {}
        for part in body.strip().split():
            if "=" not in part:
                continue
            key, val = part.split("=", 1)
            try:
                out[key] = float(val)
            except ValueError:
                pass
        return out

    def is_fresh(self, max_age_s: float = 30.0) -> bool:
        return self.last_update > 0 and (time.time() - self.last_update) <= max_age_s

    def heading_fresh(self, max_age_s: float = 5.0) -> bool:
        return self.heading_update > 0 and (time.time() - self.heading_update) <= max_age_s

    def snapshot(self) -> dict:
        return {
            "lat": self.lat,
            "lon": self.lon,
            "accuracy_m": self.accuracy_m,
            "speed_mps": self.speed_mps,
            "course_bearing_deg": self.course_bearing_deg,
            "compass_azimuth_deg": self.compass_azimuth_deg,
            "heading_fresh": self.heading_fresh(),
            "hole": self.hole,
            "fresh": self.is_fresh(),
            "course": self._course_data.get("course_name") if self._course_data else None,
            "course_error": self._load_error,
        }

    def maybe_log_yardages(self, log_cb: Callable[[str], None], min_interval_s: float = 5.0) -> None:
        if not self.is_fresh():
            return
        now = time.time()
        key = (
            self.hole,
            round(self.lat, 5),
            round(self.lon, 5),
        )
        if key == self._last_yardage_key and (now - self._last_yardage_log) < min_interval_s:
            return

        acc_txt = f"±{self.accuracy_m:.0f}m" if self.accuracy_m > 0 else ""
        if self._course_data is None or yardages_for_hole is None:
            if now - self._last_yardage_log >= min_interval_s:
                self._last_yardage_log = now
                self._last_yardage_key = key
                err = self._load_error or "set GOLF_COURSE_CACHE to a cached OSM JSON"
                log_cb(
                    f"📍 Phone GPS {self.lat:.6f},{self.lon:.6f} {acc_txt} "
                    f"hole={self.hole} (yardages: {err})"
                )
            return

        try:
            result = yardages_for_hole(self._course_data, self.hole, self.lat, self.lon)
            y = result["yardages"]
            self._last_yardage_log = now
            self._last_yardage_key = key
            pin = "pin" if result.get("pin_used") else "ctr"
            log_cb(
                f"📍 Hole {self.hole} F:{y['front']} M:{y['middle']} B:{y['back']} yds "
                f"({pin}, GPS {acc_txt})"
            )
        except Exception as e:
            if now - self._last_yardage_log >= min_interval_s:
                self._last_yardage_log = now
                log_cb(f"📍 Yardage error hole {self.hole}: {e}")

    def record_summon_fix(self, log_path: Path, event: str = "summon_start") -> None:
        """Append a GPS fix to a round JSONL file (summon logging)."""
        if not self.is_fresh():
            return
        log_path.parent.mkdir(parents=True, exist_ok=True)
        row = {
            "ts": time.time(),
            "event": event,
            "hole": self.hole,
            "lat": self.lat,
            "lon": self.lon,
            "accuracy_m": self.accuracy_m,
        }
        with log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row) + "\n")
