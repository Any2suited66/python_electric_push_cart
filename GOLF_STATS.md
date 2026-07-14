# Golf Stats Tracking — Feature Spec & TODO

Automatically capture round data on the course with minimal manual input, then export for AI analysis at home.

## Goal

Track as much of a golf round as possible without slowing play. Use **summon mode** as a natural “I’m at the ball / done with this shot” signal for GPS, and a **touchscreen prompt flow** for hole-by-hole stats. At home, sync the round file to a laptop for AI-driven analysis (trends, weak holes, GIR%, fairways, putting, etc.).

---

## Phase 1 — GPS on Summon Mode

**Trigger:** Every time summon starts (`_start_summon()` in `hoverboard_minimal.py` — double-tap Button 1 in follow-me mode).

**Capture per summon event:**
- Timestamp (UTC + local timezone)
- Lat / lon (WGS84)
- Optional: accuracy (m), altitude, speed, heading
- Hole number (inferred or assigned — see Phase 2)
- Event type: `summon_start` (and optionally `summon_complete` when rotation finishes / cart pauses)

**Hardware (Pi side):**
- [ ] **GPS receiver** — Pi Zero 2W has no built-in GPS; add an external module
- [ ] **Recommended (you have a USB hub):** USB GPS dongle (u-blox VK-172 or similar) into hub alongside ESP32 receiver — no soldering
- [ ] **Alternative:** UART module (NEO-M8N etc.) on GPIO 14/15 if you prefer not to use hub ports
- [ ] `gpsd` or `pyserial` + NMEA parser on Raspberry Pi
- [ ] Log summon events to local JSON/CSV under e.g. `~/golf_rounds/YYYY-MM-DD_HHMM.jsonl`

**USB hub layout (Pi Zero 2W):**
```
Pi micro-USB ──► USB hub
                    ├── ESP32 receiver (cart control)
                    └── USB GPS dongle (yardages + summon logging)
```
Mount the GPS dongle with a clear view of the sky (not buried inside the cart frame).

**Software hooks:**
- [ ] Call `record_summon_gps()` from `_start_summon()` (and optionally on summon complete / pause)
- [ ] Associate consecutive summon points with current hole (manual hole advance or geofence per course — later)

---

## Phase 2 — Touchscreen Hole-by-Hole Entry

### Display hardware (decided)

**Cart-mounted Pi touchscreen** — spare unit for this project:

| Spec | Value |
|------|--------|
| Size | 5 inch |
| Resolution | 800 × 480 |
| Interface | DSI (IPS LCD) |
| Touch | 5-point capacitive |
| Pi support | Pi 5 / 4B / 3B+ / 3B / B+ / A+ |
| Driver | Driver-free (kernel/firmware handles DSI + touch) |

**Mounting:** Fixed to cart frame within easy reach while walking; angled for sun glare; protect from rain/dust (optional bezel or cover when not in use).

**Pi setup checklist:**
- [ ] Connect DSI ribbon to Pi DSI port (power off first)
- [ ] Enable display in `raspi-config` or `/boot/firmware/config.txt` if needed (most “driver-free” panels work OOTB on Bookworm)
- [ ] Verify touch: `libinput` / `evtest` — should appear as a multitouch device
- [ ] Set landscape 800×480 as default (`display_rotate` in config if mounted rotated)
- [ ] Outdoor readability: max brightness, consider anti-glare film

**UI stack (Pi-side, runs alongside `hoverboard_minimal.py`):**
- [ ] **Recommended:** Local web UI (Flask/FastAPI + HTML/CSS) in kiosk mode — Chromium `--kiosk` at `http://localhost:8765`; easy wizard layouts, big buttons, works well at 800×480
- [ ] **Alternative:** Pygame or Kivy fullscreen app if you want zero browser dependency
- [ ] Golf stats UI is **separate** from the handheld controller OLED (`controller.ino`) — controller stays for driving; cart screen is for stats only

**When to prompt:** After summon completes (cart paused at ball) OR manual “Enter stats” on cart screen — configurable.

### Prompt flow (wizard, one question per screen)

Example for **Hole 1** (repeat for holes 2–18):

| Step | Prompt | UI |
|------|--------|-----|
| 1 | `Enter hole 1 score?` | **Yes** / **Skip hole** |
| 2 | `Score for hole 1?` | Keypad or scroll: 1–15+ |
| 3 | `How many putts?` | Boxes: **1** **2** **3** **4** **5+** |
| 4 | `How many chips?` | Boxes: **0** **1** **2** **3** **4+** |
| 5 | `Fairway hit?` | **Yes** / **No** / **N/A** (par 3) |
| 6 | `Green in regulation?` | **Yes** / **No** |
| 7 | `Drive distance (yards)?` | Optional: number picker or **Skip** |
| 8 | `Penalty strokes?` | **0** **1** **2** **3+** |
| 9 | `Notes?` | Optional short text or voice-to-text later |

**UX rules (800×480 layout):**
- Large tap targets — min ~80×80 px; boxed numbers 1–5 in a horizontal row (~140 px wide each where space allows)
- Prompt text: 28–36 pt; button labels: 24 pt minimum (readable outdoors)
- **Back** on every step (top-left)
- **Skip** for optional questions (drive distance, notes)
- Remember last round’s preferences (which questions to show)
- Total time target: &lt; 30 seconds per hole if skipping optional fields
- Idle screen: current hole, running score vs par, “Waiting for summon…” or cart status

### Data model (per hole)

```json
{
  "hole": 1,
  "par": 4,
  "score": 5,
  "putts": 2,
  "chips": 1,
  "fairway_hit": false,
  "gir": false,
  "drive_yards": 210,
  "penalties": 0,
  "notes": "",
  "summon_events": [
    { "ts": "2026-07-05T14:32:01-07:00", "lat": 37.123, "lon": -122.456, "type": "summon_start" }
  ]
}
```

---

## Phase 2b — Course Map & Green Distances (F / M / B)

**Goal:** On the 5″ cart display, show a hole map plus live yardages to **front**, **middle**, and **back** of the green as you walk.

### Is it possible?

**Yes.** You need two things:

1. **Your position** — GPS on the Pi (updated every second or on summon)
2. **Green geometry per hole** — polygon (or at least pin + front/back points)

Distances = great-circle math (Haversine) from you → each target point. No internet required **during the round** if course data is cached beforehand.

### Data sources (open vs paid)

| Source | Cost | Green F/M/B | Hole maps | Offline | Notes |
|--------|------|-------------|-----------|---------|-------|
| **[OpenStreetMap](https://www.openstreetmap.org/)** + Overpass API | Free (ODbL) | Derive from `golf=green` polygon | `golf=hole`, fairway, bunkers, tees | Cache GeoJSON at home / hotspot | Coverage varies — famous courses often good; many courses only have a boundary |
| **[OpenGolfAPI](https://www.opengolfapi.org/)** | Free, no API key | ❌ No refined geometry | Scorecard, par, course location | Download full US DB | ~17k US courses; schema explicitly excludes GPS green geometry |
| **[OpenCourseMaps](https://community.openstreetmap.org/t/fairwaymapper-introducing-golfers-to-mapping/142814)** | Free | ✅ If you (or others) map it | Visual OSM editor for golfers | Same as OSM | Best path to fix a missing home course |
| **[Open-Birdie](https://github.com/rroojrooj/Open-Birdie)** | Open source | From OSM greens | Full hole render | Caches courses locally | Good reference for loading OSM golf geometry on a Pi-like device |
| **[iGolf Connect](https://igolf.com/solutions/golf-course-data/)** | Commercial license | ✅ Native F/C/B points | Full GeoData, 40k+ courses | Embedded data file option | What most consumer golf GPS apps use; paid |

**Practical recommendation:** Start **open source** (OSM + Overpass), cache your courses locally, and only pay for iGolf if your home course isn’t mapped well enough.

### How front / middle / back are computed (open source)

From an OSM `golf=green` polygon per hole:

- **Middle** — `golf=pin` node if mapped, else green centroid (`shapely` / Turf.js)
- **Front** — green edge point closest to the player along the approach line (tee → pin or player → pin)
- **Back** — green edge point farthest from the player along that same axis

Python on Pi: `geopy.distance.geodesic` or `haversine`; `shapely` for polygon edge points.

```python
# Pseudocode — run on Pi with cached green polygon + live GPS fix
from geopy.distance import geodesic

player = (lat, lon)
front, middle, back = derive_green_targets(green_polygon, hole_line, player)

yards_f = geodesic(player, front).yards
yards_m = geodesic(player, middle).yards
yards_b = geodesic(player, back).yards
```

### Map UI on 800×480 display

```
┌─────────────────────────────────────┐
│  Hole 7 · Par 4        142 yds out  │
│  ┌──────────────────┐  F   M   B   │
│  │  [hole map]      │  138 152 165  │
│  │  you ●──────────►│               │
│  │       green      │  [Enter stats]│
│  └──────────────────┘               │
└─────────────────────────────────────┘
```

**Libraries:**
- **Map display:** [MapLibre GL JS](https://maplibre.org/) or [Leaflet](https://leafletjs.com/) — hole vector overlay on satellite or simple green polygon
- **Fetch OSM:** [Overpass API](https://overpass-api.de/) — query `golf=green`, `golf=hole`, `golf=pin` inside course boundary
- **Distance math:** `geopy` + `shapely` (Python backend); optional `turfpy` if you prefer Turf-style APIs
- **Offline tiles (optional):** Pre-download satellite or use vector-only hole maps to save bandwidth

### Internet / phone hotspot strategy

You do **not** need constant connectivity on the course.

1. **At home (best):** Pick course → download OSM geometry + save to `~/golf_courses/<slug>.geojson`
2. **Parking lot (hotspot):** One-time Overpass fetch if course not cached (~100 KB–2 MB per course)
3. **On course:** Pi uses local cache + live GPS only — F/M/B updates as you walk

Hotspot is enough for prefetch; skip live map tiles on course unless you want satellite imagery (heavier).

### Course-map TODO

- [ ] Overpass query script: pull hole/green/tee/fairway GeoJSON for a named course → **`golf_green_yardages.py` (POC done)**
- [ ] `derive_green_targets()` — front / middle / back from polygon + approach axis → **in `golf_green_yardages.py`**
- [ ] Cache manager — `~/golf_courses/` with manifest of downloaded courses
- [ ] Live distance endpoint — WebSocket or poll from golf-stats UI (`/api/yardages`)
- [ ] Hole map component on cart display (MapLibre or Leaflet, touch-friendly)
- [ ] Auto hole detection from GPS + hole centerlines (stretch)
- [ ] Fallback: manual hole select + “map my course” via OpenCourseMaps if OSM data is thin

---

- [ ] **Start round:** Manual “New round” or auto-detect first summon at course GPS bounds
- [ ] **Course name:** Pick from list or enter once per round
- [ ] **Tee / slope / rating:** Optional metadata for handicap context
- [ ] **Persist on Pi:** `~/golf_rounds/<date>_<course_slug>.json`
- [ ] **Crash-safe:** Append-only JSONL during round; merge to single JSON at end
- [ ] **End round:** “Finish round” → summary screen (total score, putts, FIR%, GIR%)

---

## Phase 4 — Upload & AI Analysis (at home)

- [ ] **Export:** USB copy, `scp`, or sync folder (Syncthing / rsync / simple HTTP upload page on Pi)
- [ ] **Laptop ingest:** Drop files into `golf_rounds/` for agent
- [ ] **AI agent tasks:**
  - Round summary vs par and personal averages
  - Hole-by-hole map overlay (summon GPS → approximate ball positions)
  - Trends: putting, GIR, fairways, blow-up holes
  - Suggestions: “You lose strokes on par 3s” / “GIR up 12% vs last month”
- [ ] **Format:** Stable JSON schema + README for agent tools (Cursor / custom script)

---

## Phase 5 — Automation (stretch)

- [ ] Infer hole number from course GPS map + summon sequence
- [ ] Auto par from OpenGolfAPI scorecard or OSM tags
- [ ] Live F/M/B yardages on cart map (Phase 2b)
- [ ] Drive distance from GPS: tee summon → landing summon (needs tee box markers)
- [ ] Voice prompts: “Hole 4 — how many putts?”
- [ ] Apple Watch / phone companion for quick putt-only entry
- [ ] Handicap index tracking over time

---

## Implementation order (suggested)

1. Mount 5″ DSI display on cart; verify image + touch on Pi
2. GPS logging on summon (Pi — no UI change)
3. JSON round file format + manual hole entry via SSH/debug for testing
4. Kiosk web wizard on cart display (800×480)
5. OSM course download + green F/M/B distances on map (Phase 2b)
6. Wire summon-complete → optional auto-prompt on cart screen
7. End-of-round summary on cart display
8. Home upload + AI analysis script/agent

---

## Open questions

- [x] ~~Which touchscreen hardware?~~ → **5″ 800×480 DSI capacitive, cart-mounted on Pi**
- [ ] Prompt timing: every summon vs once per hole at green?
- [ ] Course / hole map data source → **OSM + Overpass (free), cache offline; iGolf if coverage gaps**
- [ ] Privacy: keep rounds local-only vs cloud backup
- [ ] Kiosk at boot: dedicated `golf-stats.service` + Chromium, or manual launch before round?

---

## Related code today

| Piece | Location |
|-------|----------|
| Summon start | `hoverboard_minimal.py` → `_start_summon()` |
| Summon complete (paused) | `_apply_summon_rotate()` → state `paused` |
| Cart stats UI (planned) | Pi — 5″ 800×480 DSI touchscreen, kiosk web app |
| Handheld controller display | `controller/controller.ino` — SSD1306 OLED (driving only, not stats) |
| Follow-me / summon gestures | Double-tap Button 1 → summon; single tap when paused → resume |

---

*Last updated: 2026-07-05*
