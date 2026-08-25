# simulate/ — pre-hardware test suite

## Setup (one time)
```
curl -L https://wokwi.com/ci/install.sh | sh     # installs wokwi-cli
export WOKWI_CLI_TOKEN=xxxx                       # from your Wokwi CI Dashboard
```
Your `platformio.ini` needs a `wokwi_sim` env that defines `-D WOKWI_SIM`
(matches what `wokwi.toml` already points at:
`.pio/build/wokwi_sim/firmware.bin`).

## Run everything
```
./simulate/run_all.sh              # serial-only scenarios (build + run + check)
./simulate/api_endpoint_test.sh    # HTTP endpoint tests (separate — needs the port tunnel)
```
Exit code is nonzero if anything failed. Logs land in `simulate/logs/`.

## What's covered

| File | Covers |
|---|---|
| `scenarios/full_flight_test.yaml` | Boot → no-fix guard → arm → busy guard → RAISE→HOLD→MISSION → mid-mission e-stop → re-arm → full 4-waypoint mission (with a wind gust + a GPS blip mid-leg) → hover-settle → auto-land |
| `scenarios/edge_geofence_breach.yaml` | GPS jump past `GEOFENCE_RADIUS_M` (150m) → forced RTL → climb → return → settle → land |
| `scenarios/edge_gps_permanent_loss.yaml` | GPS lost with no recovery past `GPS_LOSS_ABORT_MS` (3000ms) → direct to LANDING, **never** RTL |
| `scenarios/edge_max_flight_timeout.yaml` | `MAX_FLIGHT_TIME_MS` elapses (20s in `WOKWI_SIM` builds, real 5min on hardware) → forced RTL |
| `api_endpoint_test.sh` + `scenarios/api_keepalive.yaml` | `/takeoff` `/start-waypoint-nav` `/rtl` `/abort` `/land` `/stop` `/motor-test` — guard conditions and happy path |

Each serial scenario's pass/fail rules live in `expected/<name>.expected.txt`
(`MUST:` = must appear, in order; `FORBID:` = must never appear;
`FORBID_COUNT_GT_1:` = must not repeat). `check_log.py` enforces these.

## .ino changes made to support this suite
- `MAX_FLIGHT_TIME_MS` is 20s under `WOKWI_SIM`, real 5 min otherwise.
- `physicsTick_Parked()` now actually zeroes ESC output — previously
  emergency stop reset the phase but never cut throttle.
- CRSF start/stop logic factored into `crsfHandleStart()` /
  `crsfHandleStop()`, shared by the real `crsfTask` (real hardware,
  now `#ifndef WOKWI_SIM`) and `parseSimInput()`'s new
  `CRSFSTART:1` / `CRSFSTOP:1` fake commands. **This tests the
  phase-transition logic, not the real CRSF byte-parsing** — you
  chose that tradeoff. The frame-parsing code itself only runs on
  real hardware and isn't covered by any of this.
- `parseSimInput()` gained an `ALT:<feet>` command. The sim BME280
  always returns a fixed pressure reading, so `physicsTask` no
  longer overwrites `baroAltitudeFt` under `WOKWI_SIM` — without
  this, `RTL_CLIMB`'s real-altitude exit check could never pass and
  RTL would hang forever in sim.
- Added `logLine("[WEB] Server started.")` after `server.begin()` —
  your uploaded `gps_compass_test.yaml` / `wind_gust.yaml` were
  waiting on `'[Web] Server started.'` / `'[BARO] BME280 ready.'`,
  neither of which this firmware ever printed. Fixed at the source
  instead of patching around it.

## Known gaps / things to check before flashing hardware
1. **Pin conflict**: `CRSF_RX_PIN` (GPIO16) collides with `Wire.begin(16, 15)`
   (I2C SDA). Not fixed here — I don't have `EspESC.hpp` so I can't
   confirm a free replacement pin. Check ESC pin usage, then move
   `CRSF_RX_PIN`.
2. **No live telemetry**: `lastGyroSend` / `lastAccSend` / `lastFlightSend`
   and their `SSE_*_MS` intervals are declared but never used anywhere,
   and `loop()` does nothing on a real (non-sim) build. Whatever's
   supposed to push live data to your dashboard isn't wired up.
3. **Re-arming after auto-landing**: once `PHASE_LANDED`, neither
   CRSF switch does anything (only `/stop` force-resets to `PARKED`).
   That's why `full_flight_test.yaml` puts its e-stop/re-arm test
   *before* the drone is allowed to fully land — confirm this
   "must use the ground station after landing" behavior is what you
   actually want.
4. **`api_endpoint_test.sh` is sleep-timed, not polled** — there's no
   status endpoint to poll (see gap #2). If sim timing drifts, the
   "with fix" checks could fire early/late. Fine for now; revisit if
   you add a `/status` JSON route.
5. **Not automated**: zero-waypoint `PANIC` case (needs a separate
   build with an empty `WAYPOINTS[]`), and the real CRSF byte-parser
   (needs a synthetic frame injector — you explicitly deferred this).
6. Filenames `bme280_chip.*` vs. `wokwi.toml`'s `bme280.chip.wasm` —
   double check these match on disk; likely just how they got
   uploaded to me, but worth a glance.
