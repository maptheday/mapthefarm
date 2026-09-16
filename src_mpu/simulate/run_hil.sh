#!/usr/bin/env bash
# Build, upload, and run one HIL scenario against a connected ESP32.
#
# Usage:
#   ESP_PORT=/dev/tty.usbmodem14101 ./simulate/run_hil.sh
#   ESP_PORT=/dev/tty.usbmodem14101 ./simulate/run_hil.sh edge_gps_permanent_loss.py

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PORT="${ESP_PORT:-}"
# Comment out scenarios you do not want to run.
SCENARIOS=(
    "edge_geofence_breach.py"
    "edge_gps_permanent_loss.py"
    "edge_max_flight_timeout.py"
    "full_flight_test.py"
    "manual_flight_test.py"
    "stabilization_reaction_test.py"
)

# Passing a scenario keeps the convenient single-scenario debugging mode:
#   ./simulate/run_hil.sh edge_gps_permanent_loss.py
if [[ $# -gt 0 ]]; then
    SCENARIOS=("$1")
fi
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE_BASE="$(mktemp "$LOG_DIR/hil_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
LOG_FILE="${LOG_FILE_BASE}.log"
mv "$LOG_FILE_BASE" "$LOG_FILE"

# Show output live and preserve the complete upload/test run on disk.
exec > >(tee "$LOG_FILE") 2>&1
echo "Run log: $LOG_FILE"

if [[ -z "$PORT" ]]; then
    shopt -s nullglob
    candidates=()
    for device_pattern in \
        /dev/cu.usbmodem* \
        /dev/cu.usbserial* \
        /dev/cu.SLAB_USBtoUART* \
        /dev/cu.wchusbserial*; do
        candidates+=("$device_pattern")
    done

    if [[ ${#candidates[@]} -eq 1 ]]; then
        PORT="${candidates[0]}"
        echo "Auto-discovered ESP32 port: $PORT"
    elif [[ ${#candidates[@]} -eq 0 ]]; then
        echo "ERROR: no USB serial port found." >&2
        echo "Connect the ESP32 or set ESP_PORT explicitly:" >&2
        echo "  ESP_PORT=/dev/cu.usbmodem14201 $0 [scenario.py]" >&2
        exit 2
    else
        echo "ERROR: multiple USB serial ports found:" >&2
        printf '  %s\n' "${candidates[@]}" >&2
        echo "Set ESP_PORT to select the ESP32 port." >&2
        exit 2
    fi
fi

SCENARIO_PATHS=()
for scenario in "${SCENARIOS[@]}"; do
    if [[ "$scenario" == */* ]]; then
        scenario_path="$scenario"
    else
        scenario_path="$SCRIPT_DIR/scenarios/$scenario"
    fi

    if [[ ! -f "$scenario_path" ]]; then
        echo "ERROR: scenario not found: $scenario_path" >&2
        exit 2
    fi
    SCENARIO_PATHS+=("$scenario_path")
done

if [[ -n "${PIO_BIN:-}" ]]; then
    PIO="$PIO_BIN"
elif command -v pio >/dev/null 2>&1; then
    PIO="$(command -v pio)"
elif [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
    # PlatformIO's usual macOS installation location.
    PIO="$HOME/.platformio/penv/bin/pio"
else
    echo "ERROR: PlatformIO CLI not found." >&2
    echo "Open a terminal where 'pio' works, or set PIO_BIN to its full path." >&2
    exit 2
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 not found on PATH." >&2
    exit 2
fi

cd "$PROJECT_ROOT"

echo "== Selected HIL scenarios =="
printf '  %s\n' "${SCENARIOS[@]}"

echo "== Uploading WOKWI_SIM firmware to $PORT =="
"$PIO" run -e wokwi_sim --target upload --upload-port "$PORT"

for scenario_path in "${SCENARIO_PATHS[@]}"; do
    echo "== Running HIL scenario: $scenario_path =="
    python3 "$SCRIPT_DIR/hil_runner.py" \
        --port "$PORT" \
        --scenario "$scenario_path"
    echo "== Passed HIL scenario: $scenario_path =="
done

echo "== All selected HIL scenarios passed =="
