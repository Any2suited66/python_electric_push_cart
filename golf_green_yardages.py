#!/usr/bin/env python3
"""
POC: Distance from your GPS position to front, middle, and back of the green.

Uses OpenStreetMap golf=green polygons (via Overpass API). No hole map UI —
just three yardage numbers.

Examples:
  # Oak Creek Golf Course, Irvine CA (cached in repo)
  python golf_green_yardages.py \\
    --cache golf_courses/oak_creek_irvine.json \\
    --hole 7 \\
    --lat 33.677805 --lon -117.772061

  # Live fetch (needs internet — hotspot OK in parking lot)
  python golf_green_yardages.py \\
    --course "Oak Creek Golf Course Irvine" \\
    --hole 7 \\
    --lat 33.677805 --lon -117.772061

  # Cache any course for offline rounds
  python golf_green_yardages.py \\
    --course "Oak Creek Golf Course Irvine" \\
    --fetch-only --cache golf_courses/oak_creek_irvine.json

  # Built-in synthetic green (no network)
  python golf_green_yardages.py --demo
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlencode

try:
    import requests
except ImportError:
    requests = None  # type: ignore

try:
    from shapely.geometry import LineString, Point, Polygon
except ImportError:
    Polygon = None  # type: ignore

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
NOMINATIM_URL = "https://nominatim.openstreetmap.org/search"
USER_AGENT = "pi-golf-cart-yardages/1.0"


# ---------------------------------------------------------------------------
# Geodesy (stdlib only — no geopy required)
# ---------------------------------------------------------------------------

def haversine_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def yards_between(a: tuple[float, float], b: tuple[float, float]) -> int:
    m = haversine_meters(a[0], a[1], b[0], b[1])
    return int(round(m * 1.09361))


def latlon_to_local(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(ref_lat))
    return (lon - ref_lon) * m_per_deg_lon, (lat - ref_lat) * m_per_deg_lat


def local_to_latlon(x: float, y: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(ref_lat))
    return ref_lat + y / m_per_deg_lat, ref_lon + x / m_per_deg_lon


# ---------------------------------------------------------------------------
# Green F / M / B target points
# ---------------------------------------------------------------------------

def green_ring_latlon(green_geom: list[dict[str, float]]) -> list[tuple[float, float]]:
    return [(p["lat"], p["lon"]) for p in green_geom]


def derive_green_targets(
    green_ring: list[tuple[float, float]],
    player: tuple[float, float],
    pin: tuple[float, float] | None = None,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    """
    Return (front, middle, back) as (lat, lon) from green polygon and player position.

    Middle = pin if provided, else green centroid.
    Front/back = near/far edges where the approach line (player → middle) crosses the green.
    """
    if Polygon is None:
        raise RuntimeError("shapely is required: pip install shapely")

    ref_lat = sum(p[0] for p in green_ring) / len(green_ring)
    ref_lon = sum(p[1] for p in green_ring) / len(green_ring)

    poly = Polygon([latlon_to_local(lat, lon, ref_lat, ref_lon) for lat, lon in green_ring])
    if not poly.is_valid:
        poly = poly.buffer(0)

    px, py = latlon_to_local(player[0], player[1], ref_lat, ref_lon)
    player_pt = Point(px, py)

    if pin:
        mx, my = latlon_to_local(pin[0], pin[1], ref_lat, ref_lon)
    else:
        c = poly.centroid
        mx, my = c.x, c.y

    middle_local = (mx, my)

    # On or inside the green: use nearest / farthest boundary points from player
    if poly.contains(player_pt) or poly.distance(player_pt) < 1.5:
        ring = list(poly.exterior.coords)
        front_local = min(ring, key=lambda c: math.hypot(c[0] - px, c[1] - py))
        back_local = max(ring, key=lambda c: math.hypot(c[0] - px, c[1] - py))
        return (
            local_to_latlon(front_local[0], front_local[1], ref_lat, ref_lon),
            local_to_latlon(middle_local[0], middle_local[1], ref_lat, ref_lon),
            local_to_latlon(back_local[0], back_local[1], ref_lat, ref_lon),
        )

    dx, dy = mx - px, my - py
    length = math.hypot(dx, dy)
    if length < 1.0:
        dx, dy, length = 0.0, 1.0, 1.0
    ux, uy = dx / length, dy / length

    # Long line through green along approach axis (meters local)
    behind = (px - ux * 300, py - uy * 300)
    beyond = (px + ux * 2500, py + uy * 2500)
    approach = LineString([behind, beyond])
    inter = poly.intersection(approach)

    edge_pts: list[tuple[float, float]] = []
    if inter.is_empty:
        edge_pts = list(poly.exterior.coords)
    elif inter.geom_type == "LineString":
        edge_pts = list(inter.coords)
    elif inter.geom_type == "MultiLineString":
        for seg in inter.geoms:
            edge_pts.extend(seg.coords)
    elif inter.geom_type == "Point":
        edge_pts = [(inter.x, inter.y)]
    else:
        edge_pts = list(poly.exterior.coords)

    def along_axis(pt: tuple[float, float]) -> float:
        return (pt[0] - px) * ux + (pt[1] - py) * uy

    edge_pts.sort(key=along_axis)
    front_local = edge_pts[0]
    back_local = edge_pts[-1]
    middle_local = (mx, my)

    return (
        local_to_latlon(front_local[0], front_local[1], ref_lat, ref_lon),
        local_to_latlon(middle_local[0], middle_local[1], ref_lat, ref_lon),
        local_to_latlon(back_local[0], back_local[1], ref_lat, ref_lon),
    )


def yardages_to_green(
    green_ring: list[tuple[float, float]],
    player: tuple[float, float],
    pin: tuple[float, float] | None = None,
) -> dict[str, int]:
    front, middle, back = derive_green_targets(green_ring, player, pin)
    return {
        "front": yards_between(player, front),
        "middle": yards_between(player, middle),
        "back": yards_between(player, back),
    }


# ---------------------------------------------------------------------------
# OSM / Overpass
# ---------------------------------------------------------------------------

def _require_requests() -> Any:
    if requests is None:
        raise RuntimeError("requests is required: pip install requests")
    return requests


def nominatim_golf_course(name: str) -> dict[str, Any]:
    req = _require_requests()
    params = {
        "q": name,
        "leisure": "golf_course",
        "format": "json",
        "limit": 5,
    }
    resp = req.get(NOMINATIM_URL, params=params, headers={"User-Agent": USER_AGENT}, timeout=30)
    resp.raise_for_status()
    results = resp.json()
    if not results:
        raise ValueError(f"No golf course found for: {name!r}")

    # Prefer leisure=golf_course hit whose name best matches
    def score(item: dict) -> int:
        n = (item.get("name") or "").lower()
        q = name.lower()
        s = 0
        if n == q:
            s += 100
        if q in n or n in q:
            s += 50
        if item.get("class") == "leisure" and item.get("type") == "golf_course":
            s += 25
        return s

    best = max(results, key=score)
    return best


def overpass_course_elements(osm_type: str, osm_id: int) -> list[dict]:
    req = _require_requests()
    area_stmt = f"{osm_type}({osm_id});map_to_area->.course;"
    query = f"""[out:json][timeout:60];
{area_stmt}
(
  way(area.course)["golf"="hole"];
  way(area.course)["golf"="green"];
  node(area.course)["golf"="pin"];
);
out geom;"""
    resp = req.post(
        OVERPASS_URL,
        data=urlencode({"data": query}),
        headers={
            "Content-Type": "application/x-www-form-urlencoded",
            "User-Agent": USER_AGENT,
        },
        timeout=90,
    )
    resp.raise_for_status()
    data = resp.json()
    return data.get("elements", [])


def hole_number(tags: dict[str, str]) -> str | None:
    for key in ("ref", "hole", "name"):
        val = tags.get(key)
        if val and str(val).isdigit():
            return str(val)
    return None


def hole_endpoint(hole_elem: dict) -> tuple[float, float]:
    geom = hole_elem["geometry"]
    end = geom[-1]
    return end["lat"], end["lon"]


def match_green_to_hole(hole_elem: dict, greens: list[dict]) -> dict:
    end = hole_endpoint(hole_elem)
    best = None
    best_dist = float("inf")
    for g in greens:
        ring = green_ring_latlon(g["geometry"])
        clat = sum(p[0] for p in ring) / len(ring)
        clon = sum(p[1] for p in ring) / len(ring)
        d = haversine_meters(end[0], end[1], clat, clon)
        if d < best_dist:
            best_dist = d
            best = g
    if best is None:
        raise ValueError("No greens found on course")
    if best_dist > 120:
        raise ValueError(
            f"Nearest green is {int(best_dist)}m from hole end — OSM hole/green linkage may be wrong"
        )
    return best


def match_pin_to_green(pin_nodes: list[dict], green_elem: dict) -> tuple[float, float] | None:
    if not pin_nodes:
        return None
    ring = green_ring_latlon(green_elem["geometry"])
    ref_lat = sum(p[0] for p in ring) / len(ring)
    ref_lon = sum(p[1] for p in ring) / len(ring)
    poly = Polygon([latlon_to_local(lat, lon, ref_lat, ref_lon) for lat, lon in ring])
    inside = []
    for node in pin_nodes:
        lat, lon = node["lat"], node["lon"]
        x, y = latlon_to_local(lat, lon, ref_lat, ref_lon)
        if poly.buffer(2).contains(Point(x, y)):
            inside.append((lat, lon))
    if not inside:
        # nearest pin to green centroid
        clat = sum(p[0] for p in ring) / len(ring)
        clon = sum(p[1] for p in ring) / len(ring)
        return min(
            ((n["lat"], n["lon"]) for n in pin_nodes),
            key=lambda p: haversine_meters(clat, clon, p[0], p[1]),
        )
    return inside[0]


def fetch_course_data(course_name: str) -> dict[str, Any]:
    place = nominatim_golf_course(course_name)
    osm_type = place["osm_type"][0]  # 'r' or 'w'
    osm_id = int(place["osm_id"])
    type_map = {"n": "node", "w": "way", "r": "relation"}
    elements = overpass_course_elements(type_map[osm_type], osm_id)

    holes = [e for e in elements if e.get("tags", {}).get("golf") == "hole"]
    greens = [e for e in elements if e.get("tags", {}).get("golf") == "green"]
    pins = [e for e in elements if e.get("tags", {}).get("golf") == "pin"]

    if not holes:
        raise ValueError(f"No golf=hole ways in OSM for {course_name!r}")
    if not greens:
        raise ValueError(f"No golf=green polygons in OSM for {course_name!r}")

    return {
        "fetched_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "course_name": place.get("name") or course_name,
        "osm_type": place["osm_type"],
        "osm_id": place["osm_id"],
        "bbox": place.get("boundingbox"),
        "elements": elements,
        "summary": {"holes": len(holes), "greens": len(greens), "pins": len(pins)},
    }


def load_course_data(cache_path: Path) -> dict[str, Any]:
    return json.loads(cache_path.read_text())


def save_course_data(data: dict[str, Any], cache_path: Path) -> None:
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    cache_path.write_text(json.dumps(data, indent=2))


def yardages_for_hole(
    course_data: dict[str, Any],
    hole: int,
    player_lat: float,
    player_lon: float,
) -> dict[str, Any]:
    elements = course_data["elements"]
    holes = [e for e in elements if e.get("tags", {}).get("golf") == "hole"]
    greens = [e for e in elements if e.get("tags", {}).get("golf") == "green"]
    pins = [e for e in elements if e.get("tags", {}).get("golf") == "pin"]

    hole_key = str(hole)
    hole_elem = None
    for h in holes:
        n = hole_number(h.get("tags", {}))
        if n == hole_key:
            hole_elem = h
            break
    if hole_elem is None:
        available = sorted(
            {hole_number(h.get("tags", {})) for h in holes if hole_number(h.get("tags", {}))},
            key=int,
        )
        raise ValueError(f"Hole {hole} not in OSM data. Available: {', '.join(available)}")

    green_elem = match_green_to_hole(hole_elem, greens)
    pin = match_pin_to_green(pins, green_elem)
    ring = green_ring_latlon(green_elem["geometry"])
    player = (player_lat, player_lon)
    yds = yardages_to_green(ring, player, pin)

    return {
        "course": course_data.get("course_name"),
        "hole": hole,
        "player": {"lat": player_lat, "lon": player_lon},
        "pin_used": pin is not None,
        "yardages": yds,
    }


# ---------------------------------------------------------------------------
# Demo mode (no network)
# ---------------------------------------------------------------------------

def run_demo() -> dict[str, Any]:
    # ~25m × 15m green, player ~165 yards south of front edge
    ref_lat, ref_lon = 36.5600, -121.9400
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(ref_lat))

    def offset_m(north_m: float, east_m: float) -> tuple[float, float]:
        return (
            ref_lat + north_m / m_per_deg_lat,
            ref_lon + east_m / m_per_deg_lon,
        )

    # Green rectangle centered ~200m north of origin
    green = [
        offset_m(190, -8),
        offset_m(190, 8),
        offset_m(205, 8),
        offset_m(205, -8),
        offset_m(190, -8),
    ]
    pin = offset_m(197, 0)
    player = offset_m(0, 0)  # ~190m south of green front ≈ 208 yds to front

    yds = yardages_to_green(green, player, pin)
    return {
        "course": "DEMO (synthetic green)",
        "hole": 1,
        "player": {"lat": player[0], "lon": player[1]},
        "pin_used": True,
        "yardages": yds,
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def print_result(result: dict[str, Any]) -> None:
    y = result["yardages"]
    print()
    print(f"Course:  {result.get('course', '?')}")
    print(f"Hole:    {result['hole']}")
    print(f"Player:  {result['player']['lat']:.6f}, {result['player']['lon']:.6f}")
    if result.get("pin_used"):
        print("Middle:  OSM pin (or green center)")
    else:
        print("Middle:  green center (no pin in OSM)")
    print()
    print(f"  Front:   {y['front']} yds")
    print(f"  Middle:  {y['middle']} yds")
    print(f"  Back:    {y['back']} yds")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Yardages from your GPS position to front/middle/back of the green (OSM POC)."
    )
    parser.add_argument("--course", help="Course name to search on OpenStreetMap")
    parser.add_argument("--hole", type=int, help="Hole number (1–18)")
    parser.add_argument("--lat", type=float, help="Your latitude (WGS84)")
    parser.add_argument("--lon", type=float, help="Your longitude (WGS84)")
    parser.add_argument(
        "--cache",
        type=Path,
        help="JSON cache file (load for offline, or save with --fetch-only)",
    )
    parser.add_argument(
        "--fetch-only",
        action="store_true",
        help="Download OSM data and save to --cache; do not compute yardages",
    )
    parser.add_argument("--demo", action="store_true", help="Synthetic green, no network")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    args = parser.parse_args()

    if Polygon is None:
        print("Error: install shapely — pip install shapely", file=sys.stderr)
        return 1

    try:
        if args.demo:
            result = run_demo()
            if args.json:
                print(json.dumps(result, indent=2))
            else:
                print_result(result)
            return 0

        if args.fetch_only:
            if not args.course or not args.cache:
                parser.error("--fetch-only requires --course and --cache")
            data = fetch_course_data(args.course)
            save_course_data(data, args.cache)
            s = data["summary"]
            print(f"Cached {data['course_name']} → {args.cache}")
            print(f"  holes={s['holes']} greens={s['greens']} pins={s['pins']}")
            return 0

        if args.hole is None or args.lat is None or args.lon is None:
            parser.error("--hole, --lat, and --lon are required (or use --demo)")

        if args.cache and args.cache.exists():
            course_data = load_course_data(args.cache)
        elif args.course:
            course_data = fetch_course_data(args.course)
            if args.cache:
                save_course_data(course_data, args.cache)
        else:
            parser.error("Provide --course (online) or an existing --cache file (offline)")

        result = yardages_for_hole(course_data, args.hole, args.lat, args.lon)

        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print_result(result)
        return 0

    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
