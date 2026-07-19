# Cart Follow (Android)

Custom follow-me camera app for the electric push cart. The phone tracks a person
with ML Kit pose detection and sends **steering** to the Pi over USB tethering.
The Pi reads **LiDAR distance** and sets throttle.

## Architecture

```
Pixel 6 Pro                         Raspberry Pi Zero 2W
───────────                         ────────────────────
Camera + pose tracking              Matrix LiDAR (USB-C) → throttle
Steering packets ──USB tether/TCP──► phone_bridge :9747
                                    hoverboard_minimal.py
Handheld ESP32 ────────────────────► follow-me mode on/off
```

## Packet protocol

Same 15-byte framed packet as the handheld remote (`0xAA` … `0xBB`). Phone sends:

- `steering` — lateral follow (-512 … 512)
- `throttle` — **backup** distance estimate from body size in frame; the Pi uses
  this only when the LiDAR has no valid reading
- `button_states` bit `0x80` — person detected in frame

## Distance sources (LiDAR primary, phone backup)

| Condition | Distance/throttle from |
|-----------|------------------------|
| LiDAR has a valid reading | **LiDAR** (primary, ±cm accurate) |
| LiDAR missing/no reading | **Phone** body-size estimate (coarse, ±0.3–1 m) |
| No person detected | Stop (throttle 0) |

The phone estimates distance from **torso height** (shoulder→hip) as a fraction of
frame height: bigger torso = closer. Tune `TARGET_BODY_FRAC` in `CartProtocol.kt`
for your mount height and desired follow distance. Backup throttle is capped
gentler (`MAX_BACKUP_THROTTLE = 250`) than the LiDAR path.

## Setup

### Pi

1. Plug SEN0628 **USB-C** into the Pi USB hub (no GPIO wiring).
2. Optional stable symlink: `sudo ln -sf /dev/ttyACM1 /dev/matrix-lidar` (check `ls /dev/ttyACM*`).
3. LiDAR streams 8×8 distance text at 115200 — no DIP switch or `raspi-config` I2C needed.
4. Legacy I2C: set `MATRIX_LIDAR_MODE=i2c`, wire SDA GPIO2 / SCL GPIO3, DIP = I2C.
5. Run controller:

```bash
sudo systemctl stop hoverboard-controller
python3 hoverboard_minimal.py
```

Phone-follow (TCP :9747 + LiDAR) starts automatically.

### Phone

1. Open project in Android Studio: `android/CartFollow/`
2. Build & install on Pixel 6 Pro (USB debug or sideload APK).
3. Connect phone to Pi with USB cable.
4. On phone: **Settings → Network → USB tethering** ON.
5. Open **Cart Follow**, tap **Scan & Connect to Pi** — it scans the local
   subnets for the bridge port and opens a held connection automatically.
6. On handheld remote, enable **follow-me mode**.

### Finding the Pi IP

With USB tethering the **phone is the gateway** and the Pi is a DHCP client, so its
address varies. The single **Scan & Connect** button handles this: it finds every
host with port `9747` open across the phone's subnets (tether first), then tries a
real connection to each until one holds. You can also type an IP in the box to try
it first — on the Pi, `hostname -I` shows its addresses.

All scanning and connecting runs on a background thread. (A socket call on the UI
thread throws `NetworkOnMainThreadException`, which previously made Connect fail
silently while the scan still worked — the symptom of "finds Pi but won't connect.")

## Tuning

| Constant | Location | Default |
|----------|----------|---------|
| Target follow distance | `matrix_lidar.py` | 200 cm |
| Max steering | `CartProtocol.kt` | 512 |
| Steering deadzone | `CartProtocol.kt` | 5 px |
| TCP port | both sides | 9747 |

## Development

- Pose model: ML Kit `pose-detection` (stream mode, back camera).
- Center point: hip midpoint, falls back to nose.
- Sends at 10 Hz independent of camera FPS.

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Found Pi but Connect fails | Fixed: connect now runs on a background thread (UI-thread socket calls threw `NetworkOnMainThreadException`). The app also tries every discovered host until one holds. Confirm the Pi log shows `Phone connected` and then periodic `📏 dist=…` lines (a held connection), not just quick connect/disconnect pairs (those are scan probes). |
| Connect failed | USB tethering on? Pi controller running? Firewall allows 9747 |
| `No Pi on USB :9747. phone=10.x…` | Phone USB is up; Pi is almost always on a **different** `10.x` subnet (stale `usb0`). On Pi: `ip -4 addr show usb0`, then `sudo dhclient -v usb0` (or `sudo ip addr flush dev usb0 && sudo dhclient -v usb0`). Confirm Pi ends on the **same /24** as the phone and `ss -lntp \| grep 9747` shows python listening. Prefer `scripts/usb0-dhcp-renew` so the stale address is flushed automatically. |
| Dual addresses on `usb0` | `dhclient` can keep e.g. `10.141.179.2` **and** add `10.211.203.224`. Flush (`ip addr flush dev usb0`) then renew, or use the renew script. |
| Connects from wrong IP (WiFi/cellular) | The scan tries the tether subnet first, but if the phone reaches the Pi over WiFi too, either path works. To force USB only, turn off WiFi/cellular while testing. |
| No throttle | LiDAR not aimed at torso; check USB port (`ls /dev/ttyACM*`) and mount angle |
| Steering hunts | Increase `DEADZONE_X` in `CartProtocol.kt` |
| Wrong Pi IP | Run `ip -4 addr show usb0` on Pi while tethered (not Wi‑Fi); leave app field blank for auto-scan |
| "ELF alignment check failed" / not 16 KB compatible | Android 15 16 KB page size. Use CameraX **1.4.2+** and `packaging { jniLibs { useLegacyPackaging = false } }` (already set). Then `./gradlew clean` and rebuild. |

## Android 15 / 16 KB page size

Newer Pixels (incl. Pixel 6 Pro on recent Android 15 builds) can boot with a
**16 KB memory page size**. Apps that ship native `.so` libraries aligned to the
old 4 KB boundary fail to load with *"ELF alignment check failed."*

This project avoids that by:

- **CameraX 1.4.2+** — the earlier `libimage_processing_util_jni.so` (from CameraX
  1.3.x) was only 4 KB-aligned; 1.4.2 ships the 16 KB-aligned version.
- **ML Kit pose-detection 18.0.0-beta5** — 16 KB compatible.
- **`useLegacyPackaging = false`** — keeps `.so` files uncompressed so AGP 8.5.1+
  zip-aligns them on a 16 KB boundary.

If you still hit the error: do a clean rebuild, and confirm nothing sets
`android:extractNativeLibs="false"` incorrectly or pulls in an old CameraX
transitively (`./gradlew :app:dependencies | grep camera`).
