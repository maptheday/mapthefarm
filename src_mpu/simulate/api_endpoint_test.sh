#!/usr/bin/env bash
# api_endpoint_test.sh -- curl-based tests for the REST endpoints
# (/takeoff, /start-waypoint-nav, /rtl, /abort, /land, /stop,
# /motor-test), run against a live Wokwi simulation via the
# net.forward tunnel in wokwi.toml (localhost:8180 -> sim port 80).
#
# This does NOT poll a status endpoint -- none exists yet (see
# README's SSE/telemetry note). Timing is coordinated against the
# fixed schedule in scenarios/api_keepalive.yaml via sleeps. If you
# change that yaml's delays, update the sleeps below to match.
#
# Usage:
#   WOKWI_CLI_TOKEN=xxxx ./simulate/api_endpoint_test.sh
#
# Requires: wokwi-cli, curl, WOKWI_CLI_TOKEN, PlatformIO (firmware
# already built via run_all.sh or `pio run -e wokwi_sim`).

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$SCRIPT_DIR/../wokwi.toml" ]; then
  PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
elif [ -f "$SCRIPT_DIR/wokwi.toml" ]; then
  PROJECT_ROOT="$SCRIPT_DIR"
else
  echo "ERROR: couldn't find wokwi.toml near $SCRIPT_DIR." >&2
  exit 2
fi
cd "$PROJECT_ROOT" || exit 2

if [ -z "${WOKWI_CLI_TOKEN:-}" ]; then
  echo "ERROR: WOKWI_CLI_TOKEN is not set." >&2
  exit 2
fi
if ! command -v wokwi-cli >/dev/null 2>&1; then
  echo "ERROR: wokwi-cli not found on PATH." >&2
  exit 2
fi

BASE="http://localhost:8180"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
SIM_LOG="$LOG_DIR/api_endpoint_test.log"

PASS=0
FAIL=0

# check <label> <expected_status> <curl args...>
check() {
  local label="$1" expected="$2"; shift 2
  local body status
  body="$(curl -s -o /tmp/api_test_body -w '%{http_code}' "$@")"
  status="$body"
  if [ "$status" == "$expected" ]; then
    echo "pass  [$label] status=$status"
    PASS=$((PASS + 1))
  else
    echo "FAIL  [$label] expected status=$expected, got=$status  body=$(cat /tmp/api_test_body)"
    FAIL=$((FAIL + 1))
  fi
}

echo "== Starting keepalive simulation in background =="
wokwi-cli --scenario "simulate/scenarios/api_keepalive.yaml" \
          --timeout 100000 \
          --timeout-exit-code 0 \
          --serial-log-file "$SIM_LOG" \
          . &
WOKWI_PID=$!
trap 'kill "$WOKWI_PID" 2>/dev/null' EXIT

echo "Waiting for the web server to boot..."
for i in $(seq 1 30); do
  if [ -f "$SIM_LOG" ] && grep -q '\[WEB\] Server started.' "$SIM_LOG"; then
    break
  fi
  sleep 1
done
if ! grep -q '\[WEB\] Server started.' "$SIM_LOG" 2>/dev/null; then
  echo "ERROR: sim never logged '[WEB] Server started.' after 30s -- aborting." >&2
  exit 1
fi

echo "Waiting for HTTP tunnel at $BASE to come up..."
for i in $(seq 1 20); do
  if curl -s -o /dev/null "$BASE/"; then
    break
  fi
  sleep 1
done
echo

# ── t≈0-6s window: no GPS fix yet (matches api_keepalive.yaml) ──
echo "== No-fix guard checks =="
check "takeoff, no fix -> 400"        400 "$BASE/takeoff"
check "start-waypoint-nav, PARKED -> 400" 400 "$BASE/start-waypoint-nav"
check "rtl, PARKED -> 400"            400 "$BASE/rtl"

# Wait past t=6s so the yaml's FIX:1 has landed.
echo "Waiting for GPS fix to be set by the scenario..."
sleep 7

# ── Happy path: arm, mission, abort, rtl, land, stop ──────────
echo "== Happy-path flow =="
check "takeoff, with fix -> 200"      200 "$BASE/takeoff"

echo "Waiting ~5.5s for RAISE -> HOLD (TAKEOFF_ALTITUDE_FT / RAISE_CLIMB_RATE_FPS = 15/3 = 5s)..."
sleep 6

check "start-waypoint-nav, HOLD -> 200"    200 "$BASE/start-waypoint-nav"
check "start-waypoint-nav again, MISSION -> 400 (already past HOLD)" 400 "$BASE/start-waypoint-nav"
check "motor-test while airborne -> 400"   400 "$BASE/motor-test?motor=1&pct=15"

check "abort, MISSION -> 200 (back to HOLD)" 200 "$BASE/abort"
check "rtl, HOLD -> 200"                     200 "$BASE/rtl"
check "rtl again, already RTL -> 200 (no phase guard on /rtl re-call)" 200 "$BASE/rtl"

check "land -> 200"                          200 "$BASE/land"
sleep 1
check "stop -> 200 (force back to PARKED)"   200 "$BASE/stop"

echo "== Motor-test param validation (now PARKED) =="
check "motor-test, missing params -> 400"    400 "$BASE/motor-test"
check "motor-test, motor=5 out of range -> 400" 400 "$BASE/motor-test?motor=5&pct=10"
# NOTE: not calling a valid /motor-test here on purpose -- it spins a
# real motor output for 2s. Uncomment only with props off and ESCs
# disconnected/verified safe:
# check "motor-test, valid -> 200" 200 "$BASE/motor-test?motor=1&pct=15"

echo
echo "======================================"
echo "TOTAL: $PASS passed, $FAIL failed"
kill "$WOKWI_PID" 2>/dev/null
wait "$WOKWI_PID" 2>/dev/null
if [ "$FAIL" -gt 0 ]; then
  echo "Sim log: $SIM_LOG"
  exit 1
fi
exit 0
