# HIL (Hardware-in-the-Loop) Test Runner

Connects to the real ESP32 over USB serial and runs continuous sensor loops
in background threads — barometer at ~10 Hz, GPS at ~1 Hz, compass at ~10 Hz.
Scenario scripts mutate a shared world-state dict and the sensor threads do
the rest. No Wokwi, no YAML, no race conditions.

---

## The two-step mental model

The HIL runner is just a Python script talking over a USB cable.
**The firmware has to already be on the board before you run tests.**

```
Step 1: Flash firmware  (once, or whenever you change the .ino)
Step 2: Run tests       (as many times as you want, no reflash needed)
```

---

## Step 1 — Build and flash

```bash
pio run -e wokwi_sim --target upload
```

This compiles with `WOKWI_SIM` defined, which:
- Activates `parseSimInput()` so the ESP32 reads sensor values from Serial
  instead of the real I2C sensors
- Shortens `MAX_FLIGHT_TIME_MS` to 20s for the timeout scenario

---

## Step 2 — Run tests

```bash
pip install pyserial        # one-time

./run_all_hil.sh /dev/tty.usbmodem14101
```

Or a single scenario:
```bash
python3 hil_runner.py \
  --port /dev/tty.usbmodem14101 \
  --scenario scenarios/edge_geofence_breach.py
```

Opening the serial port hardware-resets the ESP32, so every scenario
gets a clean boot automatically.

---

## The full iteration loop

```bash
# After changing firmware:
pio run -e wokwi_sim --target upload
./run_all_hil.sh /dev/tty.usbmodem14101

# After changing a scenario only:
./run_all_hil.sh /dev/tty.usbmodem14101
```

---

## How scenarios work

Each scenario is a plain Python script. It has access to:

| Function | What it does |
|---|---|
| `set_world(alt_ft=30.0, lat=36.12, ...)` | Update sensor state; threads pick it up on next tick |
| `wait_for("[NAV] Landed", timeout=30)` | Block until that string appears in firmware output |
| `forbid("[PANIC]")` | Assert a string never appeared anywhere in the log |
| `forbid_count_gt_1("[NAV] Mission complete")` | Assert something appeared at most once |
| `send("CRSFSTART:1")` | Send a raw command to the firmware |
| `arm_and_takeoff()` | Send CRSFSTART and wait for HOLD transition |
| `emergency_stop()` | Send CRSFSTOP |

The three sensor threads run continuously in the background:

| Thread | Rate | Sends |
|---|---|---|
| Barometer | 10 Hz | `ALT:<ft>` from `world["alt_ft"]` |
| GPS | 1 Hz | `FIX:`, `LAT:`, `LON:` from world state |
| Compass | 10 Hz | `HDG:<deg>` from `world["heading_deg"]` |

---

## File layout

```
hil_runner.py               # core: sensor threads, wait_for, forbid
run_all_hil.sh              # runs all four scenarios in sequence
scenarios/
  full_flight_test.py       # full 4-waypoint mission with e-stop + re-arm
  edge_geofence_breach.py   # GPS outside fence -> RTL -> land
  edge_gps_permanent_loss.py # kill fix, never restore -> direct LANDING
  edge_max_flight_timeout.py # wait for 20s timer -> RTL -> land
```

---

## Adding a new scenario

1. Create `scenarios/my_scenario.py`
2. Use `set_world()`, `wait_for()`, `forbid()`, `send()` etc.
3. Add `run_scenario "my_scenario"` to `run_all_hil.sh`
