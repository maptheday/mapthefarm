#!/usr/bin/env bash
# run_all.sh -- build the WOKWI_SIM firmware once, run every scenario in
# scenarios/*.yaml through wokwi-cli, capture each serial log, and check
# it against the matching expected/*.expected.txt manifest.
#
# Usage:
#   WOKWI_CLI_TOKEN=wok_05I9anjYhSTdkgZSP5LgzSI6vmkhlQR4a0402da8 bash ./simulate/run_all.sh
#
# Can be run from anywhere -- this script searches upward from its own
# location for platformio.ini (needed by `pio run`) and wokwi.toml
# (needed by wokwi-cli) independently, since they aren't assumed to be
# in the same directory as each other or as this script.
#
# Requires: wokwi-cli on PATH, WOKWI_CLI_TOKEN env var set, PlatformIO.
#
# api_endpoint_test.sh is intentionally NOT run from here -- it needs a
# long-lived background simulation plus curl, which doesn't fit this
# script's "run scenario to completion, then check the log" model. Run
# it separately.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Walk upward from a starting directory looking for a file. Prints the
# containing directory and returns 0 on success, prints nothing and
# returns 1 if it hits the filesystem root without finding it.
find_upward() {
  local dir="$1" target="$2"
  while [ "$dir" != "/" ]; do
    if [ -f "$dir/$target" ]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

PIO_ROOT="$(find_upward "$SCRIPT_DIR" "platformio.ini")"
if [ -z "$PIO_ROOT" ]; then
  echo "ERROR: couldn't find platformio.ini above $SCRIPT_DIR." >&2
  exit 2
fi

WOKWI_ROOT="$(find_upward "$SCRIPT_DIR" "wokwi.toml")"
if [ -z "$WOKWI_ROOT" ]; then
  echo "ERROR: couldn't find wokwi.toml above $SCRIPT_DIR." >&2
  exit 2
fi

echo "platformio.ini found at: $PIO_ROOT"
echo "wokwi.toml found at:     $WOKWI_ROOT"
echo

# wokwi.toml's firmware path ('../.pio/build/...') only resolves correctly
# if wokwi.toml's directory is exactly one level below where `pio run`
# creates .pio/ -- i.e. dirname(WOKWI_ROOT) must equal PIO_ROOT. Check
# this explicitly instead of letting wokwi-cli fail on a bad path later.
if [ "$(dirname "$WOKWI_ROOT")" != "$PIO_ROOT" ]; then
  echo "WARNING: wokwi.toml is at $WOKWI_ROOT, but platformio.ini is at $PIO_ROOT."
  echo "         wokwi.toml's 'firmware = ../.pio/build/...' path expects wokwi.toml"
  echo "         to live exactly one directory below platformio.ini. Double-check the"
  echo "         firmware/elf/fs paths in wokwi.toml match your actual layout, or"
  echo "         wokwi-cli may not find the built firmware."
  echo
fi

if [ -z "${WOKWI_CLI_TOKEN:-}" ]; then
  echo "ERROR: WOKWI_CLI_TOKEN is not set. Get one from the Wokwi CI Dashboard." >&2
  exit 2
fi
if ! command -v wokwi-cli >/dev/null 2>&1; then
  echo "ERROR: wokwi-cli not found on PATH. curl -L https://wokwi.com/ci/install.sh | sh" >&2
  exit 2
fi

LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

echo "== Building WOKWI_SIM firmware =="
if ! pio run -d "$PIO_ROOT" -e wokwi_sim; then
  echo "ERROR: firmware build failed -- fix build errors before running scenarios." >&2
  exit 1
fi
echo

PASS_COUNT=0
FAIL_COUNT=0
FAILED_NAMES=()

for scenario_path in "$SCRIPT_DIR"/scenarios/*.yaml; do
  name="$(basename "$scenario_path" .yaml)"
  expected_path="$SCRIPT_DIR/expected/$name.expected.txt"
  log_path="$LOG_DIR/$name.log"
  case "$name" in
    full_flight_test) timeout_ms=150000 ;;
    edge_geofence_breach) timeout_ms=40000 ;;
    edge_gps_permanent_loss) timeout_ms=20000 ;;
    edge_max_flight_timeout) timeout_ms=45000 ;;
    *) timeout_ms=60000 ;;
  esac

  if [ ! -f "$expected_path" ]; then
    echo "SKIP  $name -- no matching expected/$name.expected.txt, nothing to check against."
    continue
  fi

  echo "== Running scenario: $name (timeout ${timeout_ms}ms) =="
  wokwi-cli --scenario "$scenario_path" \
            --timeout "$timeout_ms" \
            --timeout-exit-code 0 \
            --serial-log-file "$log_path" \
            "$WOKWI_ROOT"
  cli_exit=$?
  if [ "$cli_exit" != "0" ]; then
    echo "  (wokwi-cli exited $cli_exit -- checking captured log anyway)"
  fi

  echo "-- Checking $name against $expected_path --"
  if python3 "$SCRIPT_DIR/check_log.py" "$expected_path" "$log_path"; then
    PASS_COUNT=$((PASS_COUNT + 1))
  else
    FAIL_COUNT=$((FAIL_COUNT + 1))
    FAILED_NAMES+=("$name")
  fi
  echo
done

echo "======================================"
echo "TOTAL: $PASS_COUNT passed, $FAIL_COUNT failed"
if [ "$FAIL_COUNT" -gt 0 ]; then
  echo "Failed: ${FAILED_NAMES[*]}"
  echo "Logs are in $LOG_DIR/ for inspection."
  exit 1
fi
exit 0