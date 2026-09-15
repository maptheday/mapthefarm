# Staged Changes Explained

This document explains the important changes currently staged in this commit.
It starts with the basic idea, then builds toward the more complicated HIL
(hardware-in-the-loop) test protocol and flight-safety behavior.

## 0. Embedded software primer for a server-side engineer

If your background is C# services on AWS, the most useful first adjustment is
to change the runtime mental model.

### A server process versus a microcontroller

In an AWS service, the operating system and cloud platform provide most of the
environment:

```text
process -> operating system -> network/socket/driver -> external service
```

The process can usually allocate memory, wait for I/O, write logs, and rely on
the OS to schedule threads. If the process exits, a supervisor can restart it.

This firmware runs directly on an ESP32 microcontroller:

```text
your C++ code -> FreeRTOS -> ESP32 CPU -> GPIO/I2C/USB/ESC hardware
```

There is no HTTP server, database, or cloud scheduler underneath the flight
controller. The code has to decide when to read sensors, how often to run
control calculations, how to share data between tasks, and how to leave the
motors in a safe state if something goes wrong.

The board also has finite resources. Memory, CPU time, serial bandwidth, and
power are all part of the design. A blocking operation that would be harmless
in a web service can make a control loop miss its deadline here.

### What an `.ino` file is

An `.ino` file is an Arduino-style C++ source file. PlatformIO preprocesses it
and compiles it into an ESP32 firmware image. The important point is that this
is still C++, not a special scripting language.

This file has two Arduino lifecycle functions:

```cpp
void setup() {
  // Runs once after the board boots.
}

void loop() {
  // Runs repeatedly after setup() finishes.
}
```

In this project, `setup()` creates mutexes, starts serial communication, and
initializes the hardware. The normal `loop()` sends web dashboard updates. In
the `WOKWI_SIM` build, `loop()` instead calls `parseSimInput()` so the Python
runner can inject sensor values over USB serial.

Unlike a C# `Main()` method that often blocks while the host handles requests,
`loop()` is expected to return quickly. Long-running work is placed in FreeRTOS
tasks. The firmware creates separate navigation and physics tasks, which run
continuously at different frequencies.

### The execution pipeline in this firmware

Read the file as a pipeline rather than as one giant class:

```text
physical sensors or HIL commands
              |
              v
       shared.raw sensor data
              |
      +-------+--------+
      |                |
      v                v
 navigationTask    physicsTask
 decides phase     computes motor mix
 and targets       and writes ESC output
      |                |
      +-------+--------+
              v
       shared state / telemetry
              |
       USB serial or web API
```

`navigationTask` answers questions such as "have we reached the takeoff
altitude?" or "did we leave the geofence?" It changes the flight phase and
the desired target altitude or steering target.

`physicsTask` answers a different question: "Given the desired target and the
current sensor readings, what should each motor receive right now?" It runs
much more often because motor stabilization needs a fast feedback loop.

This separation is similar to separating a C# orchestration service from a
high-frequency worker, except both pieces are running on the same small board
and share memory instead of communicating through a queue or database.

### C++ syntax translated from C#

The firmware uses ordinary C++ features. These are the most important ones for
reading this file:

```cpp
const unsigned long NAV_LOOP_MS = 100;
```

- `unsigned long` is an integer type that cannot represent negative values.
- `const` means the value cannot be reassigned after initialization.
- This is similar to a `const int` or `readonly` value in C#, although the
  exact integer sizes are platform-dependent in C++.

```cpp
enum FlightPhase {
  PHASE_PARKED,
  PHASE_RAISE,
  PHASE_HOLD
};
```

An `enum` is a named set of integer-like values. It is similar to a C# enum.
The code can store `PHASE_HOLD` in `shared.phase` instead of passing around a
magic number such as `2`.

```cpp
struct Cruise_Hold {
  float targetAltFt;
  float targetRollDeg;
};
```

A `struct` groups fields into a value type, similar to a simple C# `struct` or
a class containing public data. This code uses structs heavily to group raw
sensors, per-phase targets, telemetry, and mission bookkeeping.

```cpp
void transitionTo(FlightPhase next, TransitionReason reason = REASON_NONE);
```

The function declaration tells the compiler the function exists. The default
argument means both calls are valid:

```cpp
transitionTo(PHASE_LANDING);
transitionTo(PHASE_LANDING, REASON_GPS_LOSS);
```

```cpp
withMutex([&]() {
  shared.phase = PHASE_PARKED;
});
```

`[&](){ ... }` is a C++ lambda: an unnamed function value. `[&]` means it may
use local variables by reference. The lambda is passed to `withMutex`, which
locks shared state, runs the lambda, and unlocks it. In C#, the closest mental
model is passing an `Action` delegate to a helper.

```cpp
template<typename Fn>
void withMutex(Fn fn) {
  // Fn is inferred from the lambda passed by the caller.
}
```

This is a C++ template. It lets `withMutex` accept different callable types
without boxing them as a common interface. You do not need to understand
template metaprogramming to follow this use; here it is simply a type-safe
helper around a critical section.

### The preprocessor is compile-time configuration

Lines beginning with `#` are handled before C++ compilation:

```cpp
#ifdef WOKWI_SIM
const unsigned long MAX_FLIGHT_TIME_MS = 60UL * 1000UL;
#else
const unsigned long MAX_FLIGHT_TIME_MS = 5UL * 60UL * 1000UL;
#endif
```

`WOKWI_SIM` is a compile-time flag supplied by PlatformIO. The compiler sees
only one branch. It is not a runtime `if`, and the firmware cannot switch from
simulation to real hardware while running.

This is why the same source can contain both simulation input such as
`ALT:30.0` and `FIX:1`, and real I2C sensor reads from the barometer, MPU6050,
compass, and GPS.

### Timing is part of correctness

Server code often measures elapsed time with a framework timer or awaits a
`Task.Delay`. Embedded code commonly uses hardware tick counters:

```cpp
unsigned long now = millis();
if ((now - armedAtMs) >= MAX_FLIGHT_TIME_MS) {
  // Force RTL.
}
```

`millis()` returns milliseconds since boot. The subtraction form is used
because unsigned arithmetic remains well-behaved when the counter eventually
wraps. `micros()` provides finer resolution for the physics loop.

The loop periods in this firmware are deliberately different:

```text
physicsTask:     about every 5 ms  (~200 Hz)
navigationTask:  about every 100 ms (~10 Hz)
GPS HIL input:   about every 1 s
```

The physics loop needs frequent updates for stabilization. Navigation can run
more slowly because GPS movement and waypoint decisions do not need 200 updates
per second. A test that sleeps or blocks in the physics path can therefore
change the control behavior, not merely make a request slower.

### Shared memory and mutexes

The tasks share one large `SharedState` object:

```cpp
volatile SharedState shared;
SemaphoreHandle_t sharedDataMutex;

template<typename Fn>
void withMutex(Fn fn) {
  if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
    fn();
    xSemaphoreGive(sharedDataMutex);
  }
}
```

This is the embedded equivalent of protecting a shared object with a C#
`lock`. Without the mutex, one task could read a half-updated structure while
another task is writing it. For example, it might see a new latitude paired
with an old longitude.

`volatile` is not a replacement for a mutex. It tells the compiler that a
value may change outside the current code path and should not be optimized
away as if it were constant. It does not make a multi-field update atomic and
does not prevent two tasks from interleaving. The mutex provides the actual
mutual exclusion here.

There is a second mutex for serial output. Navigation, physics, and the input
parser can all produce output. `logLine()` serializes those writes and flushes
the line so two messages do not become interleaved bytes on the USB wire.

### Sensors, targets, and actuators

The control loop has three different categories of data:

1. **Measurements:** what the aircraft currently observes, such as altitude,
   GPS position, heading, and gyro rates.
2. **Targets:** what the current phase wants, such as 15 feet altitude or a
   waypoint position.
3. **Actuator commands:** the calculated motor mix sent to the four ESCs.

For example, a navigation tick may set a target altitude. A physics tick then
reads that target and the measured barometer altitude, computes a PID response,
and writes a motor mix. The HIL motor assertion checks that this last step
really happened; a phase transition by itself would not prove that the motors
were commanded correctly.

This distinction is useful when debugging embedded systems: a sensor can be
correct while the target is wrong, or both can be correct while the actuator
output is missing. The code and tests should identify which layer failed.

### A practical reading order for the `.ino` file

Do not start by reading every struct. Follow the runtime path instead:

1. Read the constants near the top: loop periods, safety limits, pins, and
  waypoint configuration. These are the system's operating assumptions.
2. Read `FlightPhase`, `TransitionReason`, and `SharedState`. These define the
  vocabulary and memory model used everywhere else.
3. Read `setup()` and `loop()`. This tells you what starts at boot and what the
  Arduino entry point does afterward.
4. Read `navigationTask()` and one `navTick_*()` function. This shows how
  measurements become phase decisions and targets.
5. Read `physicsTask()` and one `physicsTick_*()` function. This shows how
  targets and measurements become motor commands.
6. Read `transitionTo()`. This is the central state-change boundary: it carries
  state forward, initializes the next phase, and records the reason.
7. Read `parseSimInput()` last. It is an adapter for tests, not the flight
  controller itself. It translates serial text into the same state and input
  paths that the HIL runner needs to exercise.

This order is analogous to tracing an AWS request from the entry point through
the service boundary, business decision, and persistence layer. The difference
is that this request never ends: sensor/task loops keep producing new control
decisions.

## 1. How the new files fit together

The `.ino` file is now the composition root: it wires together the domain
types, configuration, sensor data, control data, and simulation state. This is
similar to a C# application's startup/composition code. It still owns the
hardware objects, FreeRTOS tasks, and flight behavior because those parts are
tightly coupled to the ESP32 runtime.

```text
FlightModel.hpp       domain vocabulary
  |
FlightConfig.hpp      timing, limits, pins, waypoints
  |
SensorTypes.hpp       measurements from real or simulated sensors
  |
ControlTypes.hpp      motor-command data structures
  |
HilState.hpp          simulation-only injected state and gates
  |
MotorController.hpp   PID state and motor/navigation control service
  |
ESP32_MPU_6050_Web_Server.ino
  hardware + tasks + navigation + physics + serial/web adapters
```

Here is the closest C# analogy, with an important embedded caveat:

| Embedded file | C# mental model | Responsibility |
|---|---|---|
| `FlightModel.hpp` | Domain enums/value vocabulary | What phases and transition reasons mean |
| `FlightConfig.hpp` | Options/constants class | Operating limits and mission configuration |
| `SensorTypes.hpp` | DTOs/records | Latest sensor measurements |
| `ControlTypes.hpp` | Command/result DTOs | Calculated four-motor output |
| `HilState.hpp` | Test adapter state | Simulation-only inputs and gates |
| `MotorController.hpp` | Control service | PID state, navigation corrections, motor mix |
| `.ino` | Composition root plus hosted workers | Hardware, scheduling, control decisions |

The headers are intentionally mostly data and pure helper functions. They do
not open serial ports, read sensors, create tasks, or decide when to land. That
makes them easier to understand and keeps side effects in the `.ino`, where
the embedded runtime is visible.

### Why these are `.hpp` files instead of `.cpp` files

Most extracted pieces are declarations, small structs, constants, enums, or
`inline` helper functions. A header is included into the `.ino` during
compilation, so the compiler sees those definitions where they are needed.

For a larger behavior-heavy module, the usual next step would be a pair:

```text
NavigationService.hpp  declarations and types
NavigationService.cpp  function implementations
```

This first extraction stops before that boundary because navigation currently
depends on global PID objects, shared state, timing, mutexes, and hardware
helpers. Moving it prematurely would create a class-shaped wrapper without
actually reducing coupling.

### A concrete data flow through the files

Suppose the drone is climbing:

1. `SensorTypes.hpp` defines the shape of the barometer and attitude readings.
2. `navigationTask()` in the `.ino` reads those measurements and updates the
   takeoff target stored in shared state.
3. `physicsTask()` reads the target and measurements, calls `MotorController`,
   and receives a `MotorMix` from `ControlTypes.hpp`.
4. The `.ino` writes that mix to the ESCs and publishes telemetry.
5. `FlightModel.hpp` supplies the phase names used to report that the drone is
   in `RAISE` or has transitioned to `HOLD`.

`HilState.hpp` participates only in the simulation build. It lets the same
navigation and physics paths receive controlled GPS, altitude, and gate input
without pretending that the Python test runner is a physical sensor.

## 2. The basic model: the drone is a state machine

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

## 3. The original testing problem: human log messages were being used as an API

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

## 4. Reliable status queries: every request gets an ID

The runner now sends commands such as:

```text
STATUS?17
```

The firmware answers with a structured line like:

```text
[STATUS] id=17 phase=RTL_CLIMB gate=NONE reason=GEOFENCE wp=0
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
[STATUS] id=17 phase=RTL_CLIMB gate=NONE reason=GEOFENCE wp=0
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
[STATUS] id=17 phase=RTL_CLIMB gate=NONE reason=GEOFENCE wp=0
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

## 5. Explicit HIL gates: tests approve expected transitions

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

## 6. Transition reasons make safety behavior observable

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

## 7. The scenarios were rewritten to test durable state

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

## 8. A small firmware behavior fix: re-arm after normal landing

The start handler now treats `LANDED` like `PARKED` for re-arming. A normally
landed aircraft is no longer considered busy, and it can begin another flight
when GPS is valid.

This was needed because the full-flight scenario intentionally stops one
mission, verifies that it did not complete, and then starts another mission.
Without this behavior, a completed landing could leave the controller refusing
the next start command even though the motors were already safely disarmed.

## 9. Runner defaults and generated files

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
