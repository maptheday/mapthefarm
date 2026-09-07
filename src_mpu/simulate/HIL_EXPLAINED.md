# HIL Testing, Explained Simply

This guide explains what the HIL test is doing, what went wrong during the geofence test, and why the final fixes work.

You do not need to understand every line of C++ or Python to use this. The main idea is:

> A Python program pretends to be the drone's sensors, while the real ESP32 runs the real flight code.

That is **Hardware-in-the-Loop**, usually shortened to **HIL**.

---

## 1. What HIL means

Normally, the drone firmware gets information from physical hardware:

- A barometer reports altitude.
- A GPS reports latitude, longitude, and GPS fix status.
- A compass reports heading.
- An RC receiver sends arming and stop commands.


During this test, we do not need to physically fly the drone. The Python program sends fake sensor values over USB instead:

## 2. The two programs

There are two programs working together:

- `src/ESP32_MPU_6050_Web_Server.ino` runs on the ESP32 and makes the flight decisions.
- `simulate/hil_runner.py` runs on the Mac and sends fake sensor values, asks questions, and checks the answers.

The scenario file, such as `simulate/scenarios/edge_geofence_breach.py`, tells the runner what situation to create and what should happen.

```text
Python test -> USB serial -> ESP32 firmware
```

For example, the Python test can send:

```text
ALT:15.00
LAT:36.123456
LON:-80.123456
FIX:1
```

The ESP32 is built with `WOKWI_SIM`. That tells the firmware:

> Read simulated sensor values from Serial instead of reading the real sensors.

The flight logic is still real. The navigation task, physics task, PID controllers, safety checks, phase transitions, and motor mixing all run on the actual ESP32.

The Python test can therefore say:

1. Start with the drone on the ground.
2. Raise its fake altitude to 15 feet.
3. Move its fake GPS position outside the geofence.
4. Check that the firmware enters RTL.
5. Raise the fake altitude to 60 feet.
6. Move the fake GPS position back to the launch point.
7. Check that the firmware lands.

Nothing actually flies, but the real flight-control decisions are exercised.

---

## 3. What the serial connection is

The USB cable is simply the conversation between the Mac and the ESP32.

Python sends a line:

```text
ALT:15.00
```

The ESP32 reads it as simulated altitude. The ESP32 can send a line back:

```text
[NAV] Takeoff altitude reached, transitioning to HOLD.
```

Python has a small waiting room called `log_queue`:

```text
ESP32 -> USB -> Python reader -> log_queue -> test
```

The queue is not another USB connection and it is not the source of the original problem. It is just where Python temporarily stores messages until the test reads them.

The important distinction is:

- **USB connection:** carries messages between the ESP32 and the Mac.
- **`log_queue`:** holds messages after Python has received them.

## 4. Why relying on log text caused trouble

The test originally waited for an exact sentence:

```python
wait_for("[NAV] Takeoff altitude reached, transitioning to HOLD.")
```

Sometimes the test saw only part of a message, such as:

```text
[NAV] Take
```

That made the test say “takeoff never happened,” even though the ESP32 may already have been in `HOLD`.

Here is the important chain of events:

1. The firmware decides that takeoff is complete.
2. The firmware changes its internal phase to `HOLD`.
3. The firmware prints a sentence to explain that decision.
4. Python waits up to one second for the complete sentence, but receives only its beginning: `[NAV] Take`.
5. The test looks for the complete sentence, does not find it, and reports a false failure.

In other words, the flight decision and the printed explanation are two separate things. The decision can be correct even when the explanation does not arrive completely.

That is why the firmware still prints logs for people, but the test asks a separate question for itself:

```text
Python:  STATUS?
ESP32:   [STATUS] phase=HOLD rtl=CLIMB
```

The log says, “Here is a sentence describing what happened.” The status response says, “Here is my state right now.” The second form is what the automated test needs.

### How can `[NAV] Take` happen technically?

The original firmware wants to send this complete line. A “line” means text followed by an invisible Enter marker:

```text
[NAV] Takeoff altitude reached, transitioning to HOLD.\n
```

The text travels from the ESP32 to Python through USB. It may be delayed along the way.

For example, the path can be thought of like this:

```text
Firmware prints -> USB connection -> Mac serial driver -> Python reader
```

Here is the simple version. In `hil_runner.py`, Python waits up to one second for each incoming line. During that second, it receives only the beginning of the firmware's sentence:

```text
Firmware sends:   [NAV] Takeoff altitude reached, transitioning to HOLD.
Python receives:  [NAV] Take
One second passes without the complete line
Python returns:   [NAV] Take
```

Why the rest was not received in that second is the low-level serial problem. It might have been delayed, separated, or lost. We cannot tell which from `[NAV] Take` alone, and you do not need to know that detail to understand the test failure. Python simply had an incomplete sentence, so it put that incomplete sentence in `log_queue`:

```text
[NAV] Take
```

The queue did not cut the message. It only stored the shortened message that Python received.

The exact low-level reason for a particular missing suffix is not visible from the text `[NAV] Take` alone. What we can say with confidence is that the test received an incomplete message, so it should not use that message as its only proof of the firmware's state.

This is similar to receiving a text message that says “The package has ar...” and never receiving the rest. The sender may have intended a complete sentence, but the receiver cannot safely assume what the missing words were.

The firmware can still continue after printing. Printing a log is not the same operation as changing the flight phase, and a damaged log does not undo the phase change.


---

## 5. Why `MOTOR?` was added

Originally, the firmware periodically printed motor telemetry without being asked. That created extra background traffic:

```text
[MOTOR] base=... roll=... pitch=...
```

At the same time, the navigation task might print:

```text
[SAFETY] Geofence exceeded -- forcing RTL.
```

The motor line also was not needed continuously. The test only needed a motor reading at specific moments.

The fix changed this from an unsolicited stream to a request/response command:

```text
Python sends:  MOTOR?
Firmware sends: [MOTOR] base=... roll=... pitch=...
```

This is better because:

- The test asks only when it needs a value.
- There is less background serial traffic.
- The test can associate a response with its own request.
- Motor telemetry is less likely to collide with safety or navigation messages.

This reduces unnecessary traffic and makes the conversation more controlled. You do not need to think of `MOTOR?` as a special USB feature. It is simply a question and answer:

```text
Python:  MOTOR?
ESP32:   [MOTOR] base=0.43 roll=0.000 pitch=-0.155
```

---

## 6. Why `STATUS?` was added

The most important fix was adding a direct state query.

The firmware already knows its actual flight phase internally:

```text
PARKED
RAISE
HOLD
MISSION
RTL
HOVER_SETTLE
LANDING
LANDED
```

The firmware now answers:

```text
Python sends:  STATUS?
Firmware sends: [STATUS] phase=RTL rtl=CLIMB
```

The Python helper uses this response:

```python
wait_for_status(phase="RTL", rtl="CLIMB", timeout=5)
```

This asks the firmware directly:

> Are you currently in RTL climb?

It no longer needs to infer that fact from a sentence such as:

```text
[SAFETY] Geofence exceeded -- forcing RTL.
```

The sentence is useful for humans. The status response is better for automation.

A good general rule is:

> Logs explain what happened to people. Status responses answer the test directly.

---

## 7. What “waiting for status” means

Here is the helper in plain English:

```python
def wait_for_status(phase=None, rtl=None, timeout=10.0):
```

It means:

> Keep asking the firmware for its state until the desired state appears, or until the deadline expires.

### Step 1: Make a deadline

```python
deadline = time.monotonic() + timeout
```

If the current time is 100 seconds and the timeout is 10 seconds, the deadline is 110 seconds.

`time.monotonic()` is used because it only moves forward. It is appropriate for measuring durations.

### Step 2: Prepare the accepted phases

```python
phases = {phase} if isinstance(phase, str) else set(phase or ())
```

This allows either one phase:

```python
phase="LANDING"
```

or several acceptable phases:

```python
phase=("HOVER_SETTLE", "LANDING", "LANDED")
```

The second form is useful when a short phase might be over by the time the test asks.

### Step 3: Keep polling until the deadline

```python
while time.monotonic() < deadline:
```

This is the loop that does the waiting.

### Step 4: Ask the firmware

```python
remaining = deadline - time.monotonic()
status = query_status(timeout=min(1.0, remaining))
```

The runner sends `STATUS?` and waits for the matching `[STATUS]` response.

The `min()` matters. Suppose only 0.2 seconds remain in the outer wait. The inner query must not wait for a full one second, or the helper could exceed its promised timeout.

### Step 5: Check this response immediately

```python
if status and condition_matches:
    return status
```

This is important. Every response is checked as soon as it arrives. The helper returns immediately when it sees the desired phase.

It does not wait until the end and inspect only the last response.

### Step 6: Pause briefly, then try again

```python
time.sleep(0.05)
```

This prevents the Python process from hammering the ESP32 with requests as fast as possible. It waits about 50 milliseconds between unsuccessful polls.

### Step 7: Report failure only after the deadline

```python
return None
```

If this happens, the firmware did not return the requested state before the timeout expired.

`None` means “no matching answer was found.” The scenario turns that into a readable assertion failure such as:

```text
Never reached takeoff altitude
```

---

## 8. Why the old status helper could fail

A previous version had the condition indented outside the polling loop. Conceptually, it behaved like this:

```python
while time.monotonic() < deadline:
    status = query_status()

# Check only one final status here
if status matches:
    return status
```

That causes three problems.

### Problem A: It can miss a fast phase

Suppose the firmware reports:

```text
RTL / CLIMB
RTL / RETURN
```

If the test only checks the last sample, it sees `RETURN` and concludes that it never saw `CLIMB`, even though `CLIMB` really happened.

### Problem B: It wastes the whole timeout

If the desired phase is received on the first poll, the broken helper still keeps polling until the deadline. That makes the test feel frozen and can add a lot of unnecessary delay.

### Problem C: It can create confusing edge behavior

Variables such as `status` may not exist if the loop never runs, for example when the timeout is zero or negative. Checking inside the loop avoids relying on a final sample that may not exist.

The current version checks each sample immediately and returns as soon as it matches.

---

## 9. The motor-reading timing race

There was another, separate problem.

When the geofence is detected, the navigation task changes the phase to RTL. The physics task runs separately and updates the motor dashboard every 5 milliseconds.

Those actions do not happen at exactly the same instant:

```text
1. Navigation changes phase to RTL.
2. Test asks for MOTOR? immediately.
3. Physics task has not updated the RTL dashboard yet.
4. Test receives the dashboard's old reset value: base=0.00.
5. Physics task runs and publishes the real climb command.
```

The first `0.00` was not proof that the motors were commanded with zero throttle. It was an old dashboard value from before the RTL physics update.

The helper now waits for a motor response whose base throttle is nonzero:

```python
wait_for_motor_base(0.05, timeout=3)
```

That gives the physics task time to publish a current sample before the test compares values.

This is a normal concurrency issue. Two independent tasks do not update shared information at the same instant just because one task changed a phase.

The firmware mutex protects the shared data from being read halfway through a write. It does not guarantee that the data is already fresh.

---

## 10. Why the HOVER_SETTLE check was adjusted

The flight path includes this sequence:

```text
RTL climb -> RTL return -> HOVER_SETTLE -> LANDING -> LANDED
```

`HOVER_SETTLE` is intentionally brief. The test asked for that exact phase, but status polling is not continuous video. It takes samples at particular times.

It is possible for the firmware to move through `HOVER_SETTLE` between two status requests:

```text
Request 1: RTL / RETURN
Firmware transitions through HOVER_SETTLE
Request 2: LANDING
```

If the test sees `LANDING`, then the drone necessarily already arrived over the launch area and completed the settle transition. Failing at that point would be backwards.

The scenario therefore accepts any of these as proof of reaching the launch area:

```python
("HOVER_SETTLE", "LANDING", "LANDED")
```

Later checks still verify the landing phase and final landed phase separately.

---

## 11. What the successful run proved

The final successful run showed:

```text
[STATUS] phase=LANDING rtl=SETTLE
RESULT: PASS
```

That means the test had already verified the important earlier steps and had reached the landing sequence. The final `LANDED` checks then completed successfully.

The test now checks the real state transitions rather than depending on every human-readable log line arriving perfectly:

```text
HOLD
RTL / CLIMB
RTL / RETURN
HOVER_SETTLE or later
LANDING
LANDED
```

The motor test also checks that the climb command is stronger than the later hover command.

---

## 12. What each kind of message is for

### Sensor input

Sent by Python to control the simulated world:

```text
ALT:30.00
LAT:36.123456
LON:-80.123456
FIX:1
HDG:90.0
```

### Control input

Sent by Python to imitate RC actions:

```text
CRSFSTART:1
CRSFSTOP:1
```

### Human-readable event log

Sent by the firmware to help a person understand what happened:

```text
[SAFETY] Geofence exceeded -- forcing RTL.
```

Useful, but not ideal as the only machine-readable test signal.

### Machine-readable response

Sent by the firmware after an explicit request:

```text
[STATUS] phase=RTL rtl=CLIMB
[MOTOR] base=0.43 roll=0.000 pitch=-0.155
```

This is what automated tests should prefer.

---

## 13. The normal workflow

When the firmware changes:

```bash
pio run -e wokwi_sim --target upload
```

Then run one scenario:

```bash
python3 simulate/hil_runner.py \
  --port "$ESP_PORT" \
  --scenario simulate/scenarios/edge_geofence_breach.py
```

When only a Python scenario or the runner changes, re-flashing is not needed. Run the scenario again with the already-running firmware.

The serial port usually looks like this on macOS:

```text
/dev/cu.usbmodem14201
```

You can set it once:

```bash
export ESP_PORT=/dev/cu.usbmodem14201
```

Opening the serial port resets the ESP32, so the test starts from a clean boot.

---

## 14. How to read a failure

### `No PING response`

The runner did not receive the firmware's ready response.

Check:

- The board is connected.
- The port is correct.
- The current firmware was flashed.
- The firmware was built with `WOKWI_SIM`.

### `Never reached takeoff altitude`

The runner did not observe `phase=HOLD` before the deadline.

Possible causes:

- Sensor commands are not reaching the firmware.
- The wrong firmware is flashed.
- The board restarted.
- The phase transition genuinely did not happen.

### `No [MOTOR] response`

The `MOTOR?` request did not receive a matching response before its timeout.

Possible causes:

- Serial traffic is overloaded.
- The firmware does not contain the new command.
- The board reset or disconnected.

### `Motor base throttle ... was not above ...`

This is a real behavior assertion. The test received motor values, but the climb value was not sufficiently higher than the later hover value.

That could indicate a firmware control problem, a sensor-timing problem, or a test assumption that needs revisiting. It is different from a missing or damaged log line.

### `RESULT: PASS`

The scenario completed all of its assertions successfully.

---

## 15. The big picture

The original failures were confusing because three different things looked similar from the outside:

1. A real firmware event happened, but its printed text was truncated.
2. A test asked for data before another task had updated it.
3. A short flight phase ended before a status poll observed it.

The fixes separate those cases:

- `STATUS?` asks for actual firmware state.
- `MOTOR?` asks for motor data only when needed.
- `wait_for_motor_base()` waits for fresh motor data.
- `wait_for_status()` polls repeatedly and checks every response.
- Accepting later phases handles short intermediate states correctly.
- The firmware keeps its output ordered, while the test no longer treats perfect logging as a requirement for proving behavior.

The simple lesson is:

> Use logs to explain what the system did. Use explicit responses and state queries to test what the system did.
