#!/usr/bin/env bash
# Run all HIL scenarios in sequence against the real ESP32.
#
# Usage: ./run_all_hil.sh /dev/tty.usbmodem14101
#
# Flash firmware first (once, or after any .ino change):
#   pio run -e wokwi_sim --target upload

set -euo pipefail

PORT="${1:?Usage: $0 <serial-port>}"
RUNNER="$(dirname "$0")/hil_runner.py"
SCENARIOS="$(dirname "$0")/scenarios"

pass=0
fail=0

run_scenario() {
    local name="$1"
    echo ""
    echo "== Running scenario: $name =="

    if python3 "$RUNNER" --port "$PORT" --scenario "$SCENARIOS/${name}.py"; then
        echo "PASS  $name"
        ((pass++)) || true
    else
        echo "FAIL  $name"
        ((fail++)) || true
    fi

    # Brief pause between scenarios -- serial open resets the ESP32,
    # this just gives it a moment to finish booting cleanly.
    sleep 3
}

run_scenario "edge_gps_permanent_loss"
run_scenario "edge_geofence_breach"
run_scenario "edge_max_flight_timeout"
run_scenario "full_flight_test"

echo ""
echo "================================"
echo "Results: $pass passed, $fail failed"
echo "================================"

[[ $fail -eq 0 ]]
