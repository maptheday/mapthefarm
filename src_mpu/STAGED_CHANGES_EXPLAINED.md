# Staged Changes Explained

This document explains the important changes currently staged in this commit.
It starts with the basic idea, then builds toward the more complicated HIL
(hardware-in-the-loop) test protocol and flight-safety behavior.

## 1. The basic model: the drone is a state machine

The firmware does not treat a flight as one large procedure. It moves through
named phases:

```text
PARKED -> RAISE -> HOLD -> MISSION -> HOVER_SETTLE -> LANDING -> LANDED
                           \-> RTL -> LANDING
```

Each phase controls important behavior: whether the motors should run, whether
the drone should follow waypoints, whether it should return home, and when it
is safe to disarm.

The tests therefore need to prove two things:

1. The firmware entered the correct phase.
2. It entered that phase for the correct safety or navigation reason.

That distinction is the foundation for the rest of the staged changes.

## 2. The original testing problem: human log messages were being used as an API

The HIL scenarios originally waited for messages such as:

```text
[NAV] Takeoff altitude reached, transitioning to HOLD.
[SAFETY] GPS fix lost - aborting directly to LANDING.
```

Those messages are useful for a person reading a flight log, but they are a
fragile control protocol for an automated test. A serial message can be split,
delayed, overwritten, or missed while the test is polling. A test can then fail
even though the firmware did the right thing, or pass based on a message that
does not prove the current state anymore.

The core fix was to give the test runner a structured state interface instead
of making it infer state from prose. This is implemented in
[hil_runner.py](simulate/hil_runner.py) and the firmware's simulation protocol
in [ESP32_MPU_6050_Web_Server.ino](src/ESP32_MPU_6050_Web_Server.ino).

## 3. Reliable status queries: every request gets an ID

The runner now sends commands such as:

```text
STATUS?17
```

The firmware answers with a structured line like:

```text
[STATUS] id=17 phase=RTL rtl=CLIMB gate=NONE reason=GEOFENCE wp=0
```

The request ID matters because serial responses can arrive late. The runner
only accepts a response whose ID matches the request it just made. Without
that correlation, a delayed answer from an earlier query could be mistaken for
the answer to a newer query.

The runner now uses one append-only log history for every complete serial line.
Lines are never removed when a helper reads them. Each helper remembers the
history length at the moment its request starts, then scans only lines appended
after that point.

For example, imagine the runner asks for motor telemetry while the firmware is
also reporting a flight-state change. The serial stream could arrive like this:

```text
[STATUS] id=17 phase=RTL rtl=CLIMB gate=NONE reason=GEOFENCE wp=0
[MOTOR] base=0.72 roll=0.03 pitch=-0.01
```

With a consuming shared queue, the test code might do this:

```python
status_line = log_queue.get()  # gets STATUS, fine
motor_line = log_queue.get()   # gets MOTOR, fine
```

But serial timing can also produce this sequence:

```text
[MOTOR] base=0.72 roll=0.03 pitch=-0.01
[STATUS] id=17 phase=RTL rtl=CLIMB gate=NONE reason=GEOFENCE wp=0
```

If `query_status()` is waiting first, it removes the motor line from the
shared queue, sees that it is not a status response, and discards it. That was
the problem with the old implementation. With the new append-only history,
`query_status()` simply skips that line in its own scan; `query_motor()` can
still find the same motor line later.

The reader now appends each line once and wakes all waiters:

```python
with log_condition:
    log_lines.append(line)
    log_condition.notify_all()
```

The request helpers use independent cursors into that history:

```python
with log_condition:
    next_line = len(log_lines)
send("MOTOR?")

# Later, after the reader appends new lines:
with log_condition:
    new_lines = log_lines[next_line:]
    next_line = len(log_lines)

for line in new_lines:
    match = _MOTOR_RE.search(line)
    if match:
        return parse_motor(match)
```

Now `query_status()` and `query_motor()` can both inspect the same response
history without consuming each other's lines. The status ID is still needed to
reject a *different status* response that arrived late; the append-only history
solves message loss, while the ID solves status-response correlation.

This design does mean the history grows for the lifetime of one scenario. That
is intentional for these short HIL runs: preserving every line is simpler and
more reliable than deleting messages after one consumer sees them. A long-lived
runner could periodically archive or truncate lines older than every active
cursor, but that is not needed for the current test process.

The status IDs and append-only history solve different problems:

- The **status ID** answers: "Is this status response for my particular
  `STATUS?17` request?"
- The **history cursor** answers: "Which lines were added after this particular
  request started?"

The current implementation makes that separation explicit:

```python
def _log_reader():
  # One serial-reader thread owns the byte stream.
  for line in _complete_lines(rx_buffer, raw):
    with log_condition:
      log_lines.append(line)
      log_condition.notify_all()

def query_status(timeout=1.0):
  request_id = next_request_id()
  start = len(log_lines)
  send(f"STATUS?{request_id}")
  while ...:
    line = log_lines[start]
    start += 1
    match = _STATUS_RE.search(line)
    if match and int(match["id"]) == request_id:
      return match.groupdict()

def query_motor(timeout=3.0):
  start = len(log_lines)
  send("MOTOR?")
  while ...:
    line = log_lines[start]
    start += 1
    match = _MOTOR_RE.search(line)
    if match:
      return parse_motor(match)
```

The real code includes the timeout and parsing details omitted from this
shortened example. The actual code takes the same snapshot and waits on a
condition variable when no new line is available. Notice that a status ID
cannot identify a `[MOTOR]` line: motor responses do not contain status IDs,
and they represent a different request type. The cursor is what keeps a
motor query from accidentally reusing an old motor response.

This is why `query_status()` and `query_motor()` are more than convenience
helpers: they prevent unrelated serial traffic from corrupting test results.

## 4. Explicit HIL gates: tests approve expected transitions

The firmware's simulation build pauses at important transitions and reports a
pending gate, for example:

```text
phase=HOLD gate=RTL reason=GEOFENCE
```

The scenario then explicitly approves that transition with:

```text
ALLOW:RTL
```

The new `approve_gate()` helper performs both halves of the check:

1. Wait until the firmware requests the expected phase and reason.
2. Approve it, then confirm that the firmware actually entered that phase and
   cleared the pending gate.

This catches both classes of defect that log matching could miss: requesting
the wrong transition and failing to complete a requested transition.

Here is the firmware code that creates and enforces that gate:

```cpp
bool hilGateAllows(FlightPhase next, TransitionReason reason) {
  if (!hilGatePending) {
    hilGatePending = true;
    hilGateApproved = false;
    hilGateNext = next;
    hilGateReason = reason;
    logLine(String("[HIL_GATE] request=") + phaseName(next) +
            " reason=" + reasonName(reason));
    return false;
  }
  return hilGateNext == next && hilGateApproved;
}

void transitionTo(FlightPhase next, TransitionReason reason) {
#ifdef WOKWI_SIM
  if (!hilGateAllows(next, reason)) return;
#endif

  // The existing transition body updates the phase and its per-phase state.
  withMutex([&]() {
    shared.phase = next;
    shared.transitionReason = reason;
  });
}
```

The approval command is handled separately:

```cpp
else if (buf.startsWith("ALLOW:")) {
  String allowed = buf.substring(6);
  for (int i = PHASE_PARKED; i <= PHASE_LANDED; ++i) {
    FlightPhase phase = static_cast<FlightPhase>(i);
    if (allowed == phaseName(phase) &&
        hilGatePending && hilGateNext == phase) {
      hilGateApproved = true;
      logLine(String("[HIL_GATE] approved=") + allowed);
      break;
    }
  }
}
```

The important detail is that `ALLOW:RTL` cannot approve an `HOLD` request:
the firmware compares the requested phase with `hilGateNext`. The runner also
checks the reason before sending approval, and checks the resulting phase after
approval:

```python
def approve_gate(phase: str, reason=None, timeout: float = 10.0):
    status = wait_for_status(gate=phase, reason=reason, timeout=timeout)
    if status is None:
        return None                 # wrong request, wrong reason, or timeout

    send(f"ALLOW:{phase}")
    return wait_for_status(phase=phase, gate="NONE", timeout=timeout)
```

So the two failure cases are checked at different points:

- **Wrong transition requested:** the first `wait_for_status()` never finds
  the expected `gate` and `reason`.
- **Requested transition not completed:** the second `wait_for_status()` never
  sees the expected `phase` with `gate=NONE`.

The gate is only a test-build synchronization mechanism. It does not make the
real aircraft wait for a test script; it makes the simulation deterministic so
the test can inspect each safety decision before allowing the state machine to
continue.

## 5. Transition reasons make safety behavior observable

The firmware now records a `TransitionReason`, including:

- `OPERATOR_START`
- `TAKEOFF_COMPLETE`
- `MAX_FLIGHT_TIME`
- `GEOFENCE`
- `GPS_LOSS`
- `MISSION_COMPLETE`
- `RTL_COMPLETE`
- `HOVER_COMPLETE`
- `TOUCHDOWN`
- `EMERGENCY_STOP`

The reason travels with the transition and is included in status responses.
That prevents a test from proving only that the drone eventually landed while
missing the fact that it landed for the wrong reason or took an unsafe route.

The safety paths now explicitly identify their causes:

- Maximum flight time requests `RTL` with `MAX_FLIGHT_TIME`.
- A geofence breach requests `RTL` with `GEOFENCE`.
- Lost GPS goes directly to `LANDING` with `GPS_LOSS`, because returning home
  requires a usable position.
- Touchdown enters `LANDED` with `TOUCHDOWN`.
- The emergency stop returns to `PARKED` with `EMERGENCY_STOP`.

The change in [ESP32_MPU_6050_Web_Server.ino](src/ESP32_MPU_6050_Web_Server.ino#L234)
is therefore an observability and correctness improvement, not just a logging
change: the test can now verify the decision that caused each phase change.

## 6. The scenarios were rewritten to test durable state

The four scenario files now use `approve_gate()` where a transition must be
caused by a particular event, and `wait_for_status()` where the test only needs
to observe stable state.

### Full flight

[full_flight_test.py](simulate/scenarios/full_flight_test.py) now verifies:

- Start is ignored without a GPS fix.
- Takeoff reaches `HOLD`.
- A second start during takeoff is ignored.
- The mission reaches all four waypoints.
- An emergency stop returns to `PARKED` and does not falsely complete the
  mission.
- The aircraft can re-arm and run the mission again.
- Mission completion enters hover settling, then automatic landing, then
  `LANDED`.

The waypoint assertions now inspect the waypoint index (`wp`) rather than
waiting for a one-time message such as "Reached waypoint 1". This proves the
current mission progress instead of relying on whether a historical log line
was observed.

### Geofence breach

[edge_geofence_breach.py](simulate/scenarios/edge_geofence_breach.py) now
requires an `RTL` gate with reason `GEOFENCE`. It also continues checking motor
output: climb throttle must rise during RTL climb, and steering corrections
must appear while returning home. This matters because a phase label alone
could hide a controller that announced RTL without actually commanding the
motors correctly.

The test accepts either durable RTL `RETURN` or `SETTLE` state because the
firmware can advance between two status polls when it is already over the
launch coordinates. This removes a timing-dependent assertion without weakening
the actual safety check.

### Permanent GPS loss

[edge_gps_permanent_loss.py](simulate/scenarios/edge_gps_permanent_loss.py)
requires `LANDING` with reason `GPS_LOSS` and forbids any RTL state. That is
important because RTL cannot safely navigate without a position fix. The test
then drives the barometer down and verifies a normal touchdown.

### Maximum flight timeout

[edge_max_flight_timeout.py](simulate/scenarios/edge_max_flight_timeout.py)
requires `RTL` with reason `MAX_FLIGHT_TIME`, then verifies the return and
landing path. This proves that the timer is a real safety mechanism rather than
only a message printed to the log.

## 7. A small firmware behavior fix: re-arm after normal landing

The start handler now treats `LANDED` like `PARKED` for re-arming. A normally
landed aircraft is no longer considered busy, and it can begin another flight
when GPS is valid.

This was needed because the full-flight scenario intentionally stops one
mission, verifies that it did not complete, and then starts another mission.
Without this behavior, a completed landing could leave the controller refusing
the next start command even though the motors were already safely disarmed.

## 8. Runner defaults and generated files

[run_hil.sh](simulate/run_hil.sh) now defaults to `full_flight_test.py`, while
the geofence, GPS-loss, and timeout defaults remain commented examples. This
makes the default command exercise the broadest scenario while keeping the
edge-case choices visible.

The staged deletion of old files under `simulate/logs/` is cleanup of captured
run output, not a flight-controller fix. Likewise, the staged `.pyc` files under
`__pycache__/` are generated Python bytecode, not source changes. They do not
explain any new behavior and are best treated separately from the functional
changes above.

When several scenarios run in one loop, the runner sends `RESET:` after each
firmware handshake. This is necessary because opening the ESP32-S3 USB serial
connection does not reliably reset the board every time. The simulation-only
reset clears the GPS fix, returns the phase to `PARKED`, clears any pending HIL
gate, and resets mission progress, so a previous scenario ending in `LANDED`
cannot make the next scenario start with stale state.

## In one sentence

The staged work changes HIL testing from fragile log-message matching into a
request-correlated, state-aware safety test system that can prove not only
where the drone went, but why it went there and whether the motors followed
the expected commands.
