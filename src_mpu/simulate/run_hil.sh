#!/usr/bin/env bash
# Build + flash the on-ESP `sim` firmware, then run the on-ESP sim scenarios.
#
# The WHOLE simulation runs on the ESP now (QuadSim physics inside the real
# 200 Hz flight loop -- no laptop in the loop, no serial jitter). Each scenario
# in scenarios_esp/ flies a mission autonomously on the chip, pulls the flight
# log back over USB (DUMPLOG), checks it, and saves the map-replay JSON.
#
# Usage:
#   ESP_PORT=/dev/cu.usbmodem14101 ./simulate/run_hil.sh
#   ESP_PORT=/dev/cu.usbmodem14101 ./simulate/run_hil.sh field_patrol.py   # one scenario
#   NO_UPLOAD=1 ESP_PORT=... ./simulate/run_hil.sh                          # skip the flash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PORT="${ESP_PORT:-}"

# The on-ESP sim scenarios to run. Comment out any you don't want. Each flashes
# nothing extra -- they all share the one `sim` build and pick their behaviour at
# boot via the SCENARIO: command. (These replace the old laptop HIL scenarios.)
SCENARIOS=(
    "field_patrol.py"      # full flight: takeoff -> whole fence line -> land (drives the Clover map)
    "geofence_breach.py"   # fly past the fence -> RTL
    "gps_loss.py"          # GPS drops -> abort straight to LANDING
    "max_timeout.py"       # flight-time limit hit -> RTL
    "manual_flight.py"     # MANUAL mode flies the drone on the sticks
    "stabilization.py"     # a gust rolls it ~22 deg -> recovers to level
)

# Passing one scenario name runs just that one:
#   ./simulate/run_hil.sh field_patrol.py
if [[ $# -gt 0 ]]; then
    SCENARIOS=("$1")
fi

LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/sim_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee "$LOG_FILE") 2>&1
echo "Run log: $LOG_FILE"

# --- find the ESP serial port ------------------------------------------------
if [[ -z "$PORT" ]]; then
    shopt -s nullglob
    candidates=(/dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.SLAB_USBtoUART* /dev/cu.wchusbserial*)
    if [[ ${#candidates[@]} -eq 1 ]]; then
        PORT="${candidates[0]}"
        echo "Auto-discovered ESP32 port: $PORT"
    elif [[ ${#candidates[@]} -eq 0 ]]; then
        echo "ERROR: no USB serial port found. Set ESP_PORT explicitly." >&2
        exit 2
    else
        echo "ERROR: multiple USB serial ports found:" >&2
        printf '  %s\n' "${candidates[@]}" >&2
        echo "Set ESP_PORT to select the ESP32 port." >&2
        exit 2
    fi
fi

# --- locate PlatformIO + the venv python -------------------------------------
if [[ -n "${PIO_BIN:-}" ]]; then
    PIO="$PIO_BIN"
elif command -v pio >/dev/null 2>&1; then
    PIO="$(command -v pio)"
elif [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
    PIO="$HOME/.platformio/penv/bin/pio"
else
    echo "ERROR: PlatformIO CLI not found (set PIO_BIN)." >&2
    exit 2
fi

PY="$SCRIPT_DIR/.venv-rotorpy/bin/python"
[[ -x "$PY" ]] || PY="python3"

# --- resolve scenario paths --------------------------------------------------
SCENARIO_PATHS=()
for scenario in "${SCENARIOS[@]}"; do
    path="$SCRIPT_DIR/scenarios_esp/$scenario"
    [[ "$scenario" == */* ]] && path="$scenario"
    if [[ ! -f "$path" ]]; then
        echo "ERROR: scenario not found: $path" >&2
        exit 2
    fi
    SCENARIO_PATHS+=("$path")
done

cd "$PROJECT_ROOT"

echo "== On-ESP sim scenarios =="
printf '  %s\n' "${SCENARIOS[@]}"

# Flash the sim firmware once (all scenarios share the one `sim` build).
if [[ "${NO_UPLOAD:-0}" != "1" ]]; then
    echo "== Flashing sim firmware to $PORT =="
    "$PIO" run -e sim --target upload --upload-port "$PORT"
fi

pass=0; fail=0
for path in "${SCENARIO_PATHS[@]}"; do
    name="$(basename "$path")"
    out="$SCRIPT_DIR/logs/${name%.py}.json"
    echo "== Running on-ESP sim: $name =="
    if "$PY" "$path" --port "$PORT" --out "$out"; then
        echo "PASS  $name  (-> $out)"; ((pass++)) || true
    else
        echo "FAIL  $name"; ((fail++)) || true
    fi
done

echo ""
echo "================================"
echo "Results: $pass passed, $fail failed"
echo "================================"
[[ $fail -eq 0 ]]
