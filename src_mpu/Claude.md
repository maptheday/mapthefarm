# Drone Firmware — A Newcomer's Guide

*Written for: someone opening this project for the first time, with little or no embedded/drone background. It starts gentle (ELI5) and gets more concrete as you go. Every file in `src/` has its own section with a link you can click.*

---

## Table of contents

1. [What this project is](#1-what-this-project-is)
2. [If you're new to embedded (a 5-minute primer)](#2-if-youre-new-to-embedded-a-5-minute-primer)
3. [The big picture (the one metaphor to remember)](#3-the-big-picture)
4. [The five core ideas](#4-the-five-core-ideas)
5. [A guided tour: one whole flight](#5-a-guided-tour-one-whole-flight)
6. [The directory map](#6-the-directory-map)
7. [File-by-file reference](#7-file-by-file-reference)
   - [Entry point](#71-entry-point)
   - [`state/` — configuration & the shared notebook](#72-state--configuration--the-shared-notebook)
   - [`models/` — plain data shapes](#73-models--plain-data-shapes)
   - [`phases/` — the flight behaviors](#74-phases--the-flight-behaviors)
   - [`services/` — the "how" (sensors, motors, math, safety)](#75-services--the-how)
   - [`hardware/` — the lowest level (chips & pins)](#76-hardware--the-lowest-level)
8. [Simulation & testing](#8-simulation--testing)
9. [Common tasks ("how do I…?")](#9-common-tasks)
10. [Glossary](#10-glossary)
11. [Honest notes & rough edges](#11-honest-notes--rough-edges)

---

## 1. What this project is

This is the **flight controller firmware** for a small **quadcopter** (a drone with 4 motors). It runs on an **ESP32-S3** — a tiny, cheap microcontroller (think: a computer the size of a stick of gum, no screen, no operating system).

The firmware's whole job is a loop that never ends:

> **read the sensors → decide what to do → spin the 4 motors the right amount → repeat, hundreds of times a second.**

It can:
- Sit safely on the ground (motors off).
- Take off to a set height when you flip a switch on a radio controller.
- Fly a pre-programmed route of GPS waypoints on its own.
- Come home by itself if something goes wrong (lost GPS, flew too far, flew too long).
- Land itself.
- Be flown by hand with RC sticks (**MANUAL** mode), including holding its position with GPS.

There's also a **simulator** so you can test all of this on your desk without a real drone (and even fly it with your keyboard — see [section 8](#8-simulation--testing)).

---

## 2. If you're new to embedded (a 5-minute primer)

A few ideas that the rest of the guide assumes. If you already know these, skip ahead.

- **Microcontroller**: a small chip that runs one program (this firmware) directly, with no operating system underneath. When you "flash" the firmware, you copy your program onto the chip and it starts running.
- **`setup()` and `loop()`**: the Arduino framework (which this uses) calls `setup()` once at power-on, then calls `loop()` over and over forever. See them at the bottom of the [main `.ino` file](#71-entry-point).
- **Two cores + "tasks"**: the ESP32 has **two CPU cores** (two brains that run at the same time). This firmware uses a system called **FreeRTOS** to run several **tasks** (mini-programs) in parallel — e.g. one task reads sensors and steers, another runs the fast motor math. Running things at the same time is powerful but dangerous: two cores writing the same data at once can corrupt it. The fix is a **mutex** (explained in [section 4](#4-the-five-core-ideas)).
- **Sensors this drone has**:
  - **IMU** (MPU6050): measures tilt and rotation — "which way am I leaning and spinning?"
  - **Barometer** (BME280): measures air pressure, which the code turns into **height**.
  - **GPS** (BN-880): measures **position** (latitude/longitude) from satellites.
  - **Compass / magnetometer** (QMC5883L): measures **heading** — "which way am I facing?"
- **The actuators**: four **ESCs** (Electronic Speed Controllers), one per motor. You send each a number from 0.0 (off) to 1.0 (full power), and it spins that motor.
- **PID controller**: the single most important control-theory idea here. It's a formula that answers "I want to be at X, I'm actually at Y — how hard should I push to fix that?" without overshooting. Fully explained at [`PID.hpp`](#pidhpp).
- **`.hpp` files**: this project is almost entirely **header files** (`.hpp`). For our purposes think of each as "one module, fully contained in one file." The `#pragma once` at the top just means "don't paste me in twice."

---

## 3. The big picture

Here's the one mental model that makes everything else click:

> **Think of the drone as a person doing a job, using a shared notebook.**

- **The notebook** = one big shared data structure called `shared` (in [`PhaseState.hpp`](#phasestatehpp)). Everything the drone knows — sensor readings, what it's aiming for, which mode it's in — is written in this notebook.
- **The senses** = the **services** (`Imu`, `Gps`, `Compass`, `Altimeter`). They read the physical chips and write fresh numbers into the notebook.
- **The current job** = the **phase** (PARKED, TAKEOFF, MISSION, LANDING, MANUAL, …). Only one phase is "active" at a time. The phase reads the notebook, decides what to aim for, and writes commands.
- **The hands** = the **Motors** service, which takes the phase's decision and actually spins the props.
- **The pen rule** = the **mutex**. Because two cores share the one notebook, you must "hold the pen" (`withMutex`) whenever you read or write it, so two writes never smear together.

Everything in the codebase is one of those roles. When you open a new file and wonder "what is this?", ask: *is it a sense, the notebook, a job, the hands, or the plumbing that connects them?*

```
        SENSORS (services)                 THE NOTEBOOK                 THE JOB (phase)         THE HANDS
   Imu / Gps / Compass / Altimeter  ──►   shared state   ──►    ParkedPhase / RaisePhase / ...  ──►  Motors ──► 4 ESCs
        "what's true right now"          (guarded by a         "given that, what do I aim for      "spin the
                                          mutex 'pen')          and how hard do I push?"            props"
```

---

## 4. The five core ideas

Understand these five and you understand the architecture. Every file is an instance of one of them.

### Idea 1 — Two loops on two cores

The drone thinks at **two speeds**, set up in the [main `.ino`](#71-entry-point):

| Task | How often | Job |
|---|---|---|
| **navigation task** | every 100 ms (**10 Hz**) | slow "where am I going" thinking: read GPS/compass, pick targets, run failsafes, switch phases |
| **physics task** | every 5 ms (**200 Hz**) | fast "keep me stable" thinking: run the PID + motor math and drive the motors |

Why two? Steering to a waypoint doesn't need to happen 200 times a second, but *staying upright* does. Splitting them means the fast stability loop is never slowed down by the slow navigation loop. The timing numbers live in [`FlightConfig.hpp`](#flightconfighpp) (`NAV_LOOP_MS`, `PHYSICS_LOOP_MS`).

### Idea 2 — One shared notebook, guarded by a "pen"

All state lives in a single `shared` structure ([`PhaseState.hpp`](#phasestatehpp)). Any code that touches it must wrap the access in `withMutex(...)`:

```cpp
withMutex([&]() { phase = shared.phase; });   // read safely
```

The mutex is "the pen" — only one core can hold it at a time, so the nav task and physics task never corrupt each other's writes. **Rule: never touch `shared` outside `withMutex`, and never nest two `withMutex` calls** (the pen isn't re-entrant — grabbing it twice would deadlock).

### Idea 3 — Phases are swappable "job cards"

A **flight phase** is a self-contained behavior (takeoff, mission, landing…). They all follow the same tiny contract, [`IFlightPhase.hpp`](#iflightphasehpp), which has just three methods:

- `onEnter()` — run once when this phase begins ("set up my desk").
- `navTick()` — run 10×/second (slow thinking + deciding when to switch phases).
- `physicsTick()` — run 200×/second (fast motor math).

The key design choice (this comes from the project's own rule): **phases share no code with each other.** There's no common base class doing work — each phase file is complete on its own. A bug in MISSION physically cannot leak into LANDING. You can read one phase file top-to-bottom and understand it alone. (This is a deliberate preference in this project; see [section 11](#11-honest-notes--rough-edges).)

### Idea 4 — Services hide the "how"

A **service** wraps one messy real-world thing behind a clean method. The phases never talk to a GPS chip's raw bytes; they just read `shared.raw.gps.lat`, which the [`Gps`](#gpshpp) service filled in. Motors are the same: a phase computes a `MotorMix` and hands it to [`Motors`](#motorshpp) — nothing else in the codebase touches motor pins. This is why you can swap real hardware for a simulator without the flight logic noticing.

### Idea 5 — One codebase, two worlds (`WOKWI_SIM`)

The same code runs on a **real drone** and in a **desk simulator**. A compile-time switch, `WOKWI_SIM`, decides which. You'll see this pattern everywhere:

```cpp
#ifdef WOKWI_SIM
   // pretend: numbers are injected over USB by a test harness
#else
   // real: read the actual sensor chip
#endif
```

In sim mode there are no chips, so a Python test harness (or our keyboard UI) feeds fake sensor readings over the USB cable, and the motor outputs go nowhere. The *flight logic in between is identical*. This is what makes the drone testable. See [section 8](#8-simulation--testing).

---

## 5. A guided tour: one whole flight

The best way to learn the code is to follow what happens, in order, during a real flight. Each step links to the file that does the work.

1. **Power on.** The chip runs `setup()` in [`ESP32_MPU_6050_Web_Server.ino`](src/ESP32_MPU_6050_Web_Server.ino). It creates the two "pens" (mutexes), brings up each sensor via its service's `begin()`, and starts the nav + physics + radio tasks. The drone starts in **PARKED** (motors off) — that's the default in [`PhaseState.hpp`](src/state/PhaseState.hpp).

2. **Idle in PARKED.** [`ParkedPhase`](src/phases/ParkedPhase.hpp) does almost nothing except keep disarming the motors every tick (belt-and-suspenders safety) and refresh the dashboard numbers.

3. **You flip the START switch** on the radio. The [`RcInput`](src/services/RcInput.hpp) service sees it and calls `crsfHandleStart()`. If there's a GPS fix, it switches the drone to **RAISE** (takeoff).

4. **The switch happens** inside `transitionTo()` in [`PhaseMachine.hpp`](src/phases/PhaseMachine.hpp). This is the *only* place phases change. It builds a little "entry context" (current height, heading, position) and calls the new phase's `onEnter()`.

5. **Takeoff (RAISE).** [`RaisePhase`](src/phases/RaisePhase.hpp) slowly raises its *target altitude* from 0 to 15 ft. Each fast tick it calls the [`MotorController`](src/services/MotorController.hpp), which runs a PID and produces four motor numbers, handed to [`Motors`](src/services/Motors.hpp). When it reaches height, it switches to **HOLD**.

6. **Hover (HOLD).** [`HoldPhase`](src/phases/HoldPhase.hpp) just holds altitude and waits. It also runs the **failsafes** ([`Failsafes.hpp`](src/services/Failsafes.hpp)) every tick — the safety net that forces a return-home if you fly too long, too far, or lose GPS.

7. **You flip START again** → the mission begins. [`MissionPhase`](src/phases/MissionPhase.hpp) reads the waypoint list from [`FlightConfig.hpp`](src/state/FlightConfig.hpp), uses [`NavMath`](src/services/NavMath.hpp) to work out "how far and which way to the next point," tilts the drone to fly there, and advances through the list.

8. **Mission done → HOVER_SETTLE → LANDING.** [`HoverSettlePhase`](src/phases/HoverSettlePhase.hpp) pauses to bleed off momentum, then [`LandingPhase`](src/phases/LandingPhase.hpp) walks the target altitude back down to the ground.

9. **Touchdown (LANDED).** [`LandedPhase`](src/phases/LandedPhase.hpp) cuts the motors. It behaves like PARKED but means "finished a flight." START can re-arm from here.

10. **At any point: STOP switch** → instantly back to PARKED, motors cut ([`RcInput`](src/services/RcInput.hpp) `crsfHandleStop()`). And if a failsafe trips mid-flight, the drone comes home on its own: [`RtlClimbPhase`](src/phases/RtlClimbPhase.hpp) → [`RtlReturnPhase`](src/phases/RtlReturnPhase.hpp) → [`RtlSettlePhase`](src/phases/RtlSettlePhase.hpp) → LANDING (three ordinary phases, each handing off to the next).

If you read those ten files in that order, you'll have seen ~80% of the system working together.

---

## 6. The directory map

```
src/
├── ESP32_MPU_6050_Web_Server.ino   ← START HERE: the entry point, wires everything together
│
├── state/          the drone's memory & configuration
│   ├── FlightConfig.hpp            all the tunable numbers & pins (compile-time)
│   ├── PhaseState.hpp              the shared "notebook" + the mutex "pen"
│   └── HilState.hpp                sim-only fake-sensor globals
│
├── models/         plain data shapes (no logic)
│   ├── FlightModel.hpp             the list of phases & transition reasons (+ their names)
│   ├── SensorTypes.hpp             shapes for raw sensor readings & RC sticks
│   └── ControlTypes.hpp            the MotorMix shape (4 motor numbers)
│
├── phases/         the flight behaviors (one file each)
│   ├── IFlightPhase.hpp            the 3-method contract every phase implements
│   ├── PhaseSwitch.hpp             forward-declaration of transitionTo() (breaks a cycle)
│   ├── PhaseMachine.hpp            transitionTo(): the ONE place phases change
│   ├── PhaseRegistry.hpp           the lookup table: phase enum → phase object
│   ├── ParkedPhase.hpp             on the ground, motors off
│   ├── RaisePhase.hpp              takeoff climb
│   ├── HoldPhase.hpp               hover & wait (runs failsafes)
│   ├── MissionPhase.hpp            fly the GPS waypoint route
│   ├── HoverSettlePhase.hpp        pause after the last waypoint
│   ├── LandingPhase.hpp            controlled descent
│   ├── LandedPhase.hpp             touched down, motors cut
│   ├── RtlClimbPhase.hpp           RTL step 1: climb to a safe altitude
│   ├── RtlReturnPhase.hpp          RTL step 2: fly back over launch
│   ├── RtlSettlePhase.hpp          RTL step 3: settle, then land
│   ├── CalibratePhase.hpp          ground maintenance: compass calibration
│   └── ManualPhase.hpp             fly by RC sticks (+ GPS position hold)
│
├── services/       the "how": sensors, motors, math, safety, radio, logging
│   ├── PID.hpp                     the P-I-D control formula
│   ├── MotorController.hpp         PIDs + motor mixing (targets → 4 motor numbers)
│   ├── Motors.hpp                  the only thing that talks to the 4 ESCs
│   ├── NavMath.hpp                 GPS geometry (distance, bearing, N/E split)
│   ├── Failsafes.hpp               the safety net (timeout / geofence / GPS loss)
│   ├── RcInput.hpp                 the radio (CRSF/ELRS): sticks + switches
│   ├── SimAdapter.hpp              sim-only: the USB test protocol
│   ├── Log.hpp                     thread-safe serial printing + PANIC()
│   ├── Imu.hpp                     accel/gyro → attitude (MPU6050 + Madgwick)
│   ├── Gps.hpp                     GPS receiver (BN-880 via TinyGPSPlus)
│   ├── Compass.hpp                 magnetometer + calibration (QMC5883L)
│   └── Altimeter.hpp               barometer → height above ground (BME280)
│
└── hardware/       the lowest level: individual chips & pins
    ├── IESC.hpp                    interface: "an ESC" (see note in §11)
    ├── EspESC.hpp                  DShot600 motor driver (ESP32 RMT peripheral)
    ├── IBarometer.hpp              interface: "a barometer"
    └── EspBarometer.hpp            BME280 barometer driver
```

> **Reading order for a newcomer:** the [`.ino`](#71-entry-point) → [`FlightConfig.hpp`](#flightconfighpp) → [`PhaseState.hpp`](#phasestatehpp) → [`IFlightPhase.hpp`](#iflightphasehpp) → [`ParkedPhase.hpp`](#parkedphasehpp) → [`RaisePhase.hpp`](#raisephasehpp) → [`MotorController.hpp`](#motorcontrollerhpp) → [`PID.hpp`](#pidhpp). After that, the rest falls into place.

---

## 7. File-by-file reference

Every file below has: a one-line **ELI5**, what it does, and how it connects to the rest.

### 7.1 Entry point

#### [`ESP32_MPU_6050_Web_Server.ino`](src/ESP32_MPU_6050_Web_Server.ino)
**ELI5:** the front door — it's where everything is created and started.

This is the "composition root": the one file that owns the real instances of every service (`motors`, `compass`, `gps`, `imu`, `altimeter`, `motorController`) and the shared notebook (`shared`) and its two mutexes. Every other file refers to these as `extern` ("it lives somewhere else, trust me"); they are actually *born* here, once.

Key parts:
- `setup()` — creates the mutexes, opens the serial port, brings up each sensor (real builds only), and launches the tasks.
- `navigationTask()` — the 10 Hz loop: refresh GPS/compass into `shared`, then call the current phase's `navTick()`.
- `physicsTask()` — the 200 Hz loop: refresh IMU/barometer into `shared`, then call the current phase's `physicsTick()`.
- `loop()` — on real hardware, does nothing (all work is in tasks); in sim, it runs `parseSimInput()` to receive fake sensor data over USB.

The two tasks are pinned to different cores (`xTaskCreatePinnedToCore`). Notice the `#ifdef WOKWI_SIM` blocks: in sim, GPS/IMU/baro come from injected values instead of chips.

> **Historical note:** the filename says "Web_Server" but there's no web server in this code anymore. It's a leftover name. (The project owner prefers a lightweight local UI instead — see [`simulate/manual_ui.html`](#8-simulation--testing).)

---

### 7.2 `state/` — configuration & the shared notebook

#### [`FlightConfig.hpp`](src/state/FlightConfig.hpp)
**ELI5:** the settings menu — every number you might want to tweak, in one place.

No logic here, just named constants: loop timing, the max flight time, geofence radius, takeoff/RTL altitudes, the **waypoint list**, GPS pins, and the CRSF radio channel map + the MANUAL-mode tuning (lean angle, climb rate, yaw rate, deadband, position-hold on/off). If you want to change *how the drone behaves* without changing *how it works*, you almost always come here. Note the values differ between sim and real (e.g. `MAX_FLIGHT_TIME_MS` is 1 minute in sim, 5 minutes real).

#### [`PhaseState.hpp`](src/state/PhaseState.hpp)
**ELI5:** the shared notebook, and the pen rule for writing in it.

The single most important state file. It defines:
- Three little structs **per phase**: `Dashboard_X` (numbers to show), `Cruise_X` (what the phase is aiming for), `Trip_X` (private working memory). The header comment has a great rule of thumb for which to use.
- `SharedState` — the whole notebook, bundling every phase's blocks plus `raw` (latest sensor readings), the current `phase`, and the RC `sticks`.
- `withMutex(fn)` — "grab the pen, run your code, put the pen back." **Always** used when touching `shared`.

The many `operator=(const volatile ...&)` bits are a C++ detail: `shared` is marked `volatile` (it changes across cores), and these let you copy a block out of it cleanly.

#### [`HilState.hpp`](src/state/HilState.hpp)
**ELI5:** the sim's fake sensors and the "may I change phase?" gate — only exists in sim builds.

Sim-only global variables: the injected GPS (`simGpsLat/Lon/Fix`), compass heading, and the **HIL gate** flags. The gate is a clever test feature: in sim, phase changes don't happen instantly — they *pause and wait for the test harness to approve them* (`ALLOW:`), so a test can observe and control every transition. On real hardware there's no gate; transitions are immediate.

---

### 7.3 `models/` — plain data shapes

These three files contain zero logic — just the "shapes" of data that flow around.

#### [`FlightModel.hpp`](src/models/FlightModel.hpp)
**ELI5:** the master list of every flight mode and every reason it might switch.

Defines the `FlightPhase` enum (PARKED, RAISE, HOLD, MISSION, RTL_CLIMB, RTL_RETURN, RTL_SETTLE, HOVER_SETTLE, LANDING, LANDED, CALIBRATE, MANUAL), the `TransitionReason` enum (why a switch happened, for logging), and helper functions that turn each into a readable string. **Important gotcha, stated in the file:** new phases must be added at the *end* of the enum, because the registry table ([`PhaseRegistry.hpp`](#phaseregistryhpp)) is indexed by this order.

#### [`SensorTypes.hpp`](src/models/SensorTypes.hpp)
**ELI5:** the shapes for "a GPS reading," "an IMU reading," and "the RC sticks."

Structs: `RawImuReading` (tilt/spin/accel), `RawGpsReading` (lat/lon/fix/sats), `RawSticks` (throttle/roll/pitch/yaw from the radio), and `RawSensors` which bundles them together. These are what the services fill in and the phases read.

#### [`ControlTypes.hpp`](src/models/ControlTypes.hpp)
**ELI5:** the shape of a motor command — four numbers plus a few extras for display.

Just `MotorMix`: `m1, m2, m3, m4` (the four motor throttles, 0–1) plus `baseThrottle`, `rollCorrection`, `pitchCorrection` (kept for the dashboard so you can see *why* the motors are doing what they're doing). Produced by [`MotorController`](#motorcontrollerhpp), consumed by [`Motors`](#motorshpp).

---

### 7.4 `phases/` — the flight behaviors

First the four "framework" files, then the ten actual behaviors.

#### [`IFlightPhase.hpp`](src/phases/IFlightPhase.hpp)
**ELI5:** the plug shape every flight mode must fit into.

The interface (like a C# `interface`). A phase implements `id()`, and any of `onEnter()`, `navTick()`, `physicsTick()` (unused ones default to doing nothing). Also defines `EnterContext` — the little bundle of "where we are right now" that `transitionTo()` fills in and hands to a phase's `onEnter()`. The header comment is worth reading; it explains the deliberate no-shared-behavior design.

#### [`PhaseSwitch.hpp`](src/phases/PhaseSwitch.hpp)
**ELI5:** a one-line promise that "a function called `transitionTo` exists somewhere."

The only forward-declaration in the project. It exists to break a chicken-and-egg cycle: phases call `transitionTo()`, but `transitionTo()` calls back into phases. A phase includes this tiny header to learn the function's *shape* without pulling in its whole body. The real body is in `PhaseMachine.hpp`.

#### [`PhaseMachine.hpp`](src/phases/PhaseMachine.hpp)
**ELI5:** the gearbox — the single place where the drone shifts from one mode to another.

Contains the real `transitionTo(next, reason)`. It builds the `EnterContext`, carries forward "trip" info like arm-time and launch point, calls the new phase's `onEnter()`, and records the new phase. In sim, it also runs the **HIL gate** (pause-and-wait-for-approval). If you ever wonder "how does the drone actually change modes?" — it's here, and *only* here.

#### [`PhaseRegistry.hpp`](src/phases/PhaseRegistry.hpp)
**ELI5:** the phone book — given a phase name, hand back the object that runs it.

Holds one shared instance of each phase and a `table[]` that maps the `FlightPhase` enum to the right object via `phaseFor(phase)`. **The table order must exactly match the enum order** in `FlightModel.hpp` — this is the reason new phases go at the end.

Now the ten behaviors. They share a **common rhythm** (learn it once, and every phase reads the same): `onEnter()` sets the initial targets; `navTick()` updates the dashboard and decides whether to switch phases; `physicsTick()` copies the targets + latest sensors, calls `motorController.computeMotorMix(...)`, sends the result to `motors.writeMix(...)`, and records the mix for display.

#### [`ParkedPhase.hpp`](src/phases/ParkedPhase.hpp)
**ELI5:** sitting on the ground, motors dead, waiting for START.
The safe resting state and the emergency-stop destination. Its `physicsTick()` calls `motors.disarmAll()` *every tick* so a stop never relies on a stale command.

#### [`RaisePhase.hpp`](src/phases/RaisePhase.hpp)
**ELI5:** takeoff — smoothly raise the target height, then hand off to HOLD.
Ramps `targetAltFt` from 0 up to `TAKEOFF_ALTITUDE_FT` at a fixed climb rate, keeping attitude level. Records the arm-time and launch position (needed later for RTL). Switches to HOLD when it reaches height.

#### [`HoldPhase.hpp`](src/phases/HoldPhase.hpp)
**ELI5:** hover in place and wait for the next command.
Holds altitude and heading. Crucially, this is where the core **failsafes** run each tick (`checkCoreFailsafes`). START from HOLD begins the mission.

#### [`MissionPhase.hpp`](src/phases/MissionPhase.hpp)
**ELI5:** fly the pre-set GPS route, point to point.
Each nav tick: run failsafes, compute distance + bearing to the current waypoint using [`NavMath`](#navmathhpp), turn to face it, tilt to move toward it (the tilt amounts come from the nav PIDs), and advance to the next waypoint when close enough. After the last one → HOVER_SETTLE.

#### [`HoverSettlePhase.hpp`](src/phases/HoverSettlePhase.hpp)
**ELI5:** pause and steady up after the last waypoint before landing.
Hovers for a fixed time (`MISSION_COMPLETE_HOVER_MS`) to bleed off momentum, then → LANDING.

#### [`LandingPhase.hpp`](src/phases/LandingPhase.hpp)
**ELI5:** come down gently, then cut the motors.
Walks `targetAltFt` down at `LAND_DESCENT_RATE_FPS` until it reaches 0, then → LANDED. Also the direct abort target when GPS is lost (can't RTL without GPS).

#### [`LandedPhase.hpp`](src/phases/LandedPhase.hpp)
**ELI5:** touched down — like PARKED, but "we just finished flying."
Motors cut. START can re-arm from here.

#### The RTL trio: [`RtlClimbPhase.hpp`](src/phases/RtlClimbPhase.hpp) · [`RtlReturnPhase.hpp`](src/phases/RtlReturnPhase.hpp) · [`RtlSettlePhase.hpp`](src/phases/RtlSettlePhase.hpp)
**ELI5:** the "uh-oh, come home" autopilot — three ordinary phases, run back-to-back.
A failsafe (max flight time / geofence) drops the drone into **RTL_CLIMB** (rise to `RTL_ALTITUDE_FT`), which hands off to **RTL_RETURN** (fly back over the launch point using the same GPS nav as MISSION), which hands off to **RTL_SETTLE** (hover for `RTL_SETTLE_MS` to bleed off momentum) → LANDING. Each is a normal, self-contained, individually-triggerable phase with its own `Dashboard_/Cruise_/Trip_` block — **not** a "phase inside a phase." The launch point is carried forward from one step to the next by `transitionTo()` (see the `case PHASE_RTL_CLIMB` / `case PHASE_RTL_RETURN` blocks in [`PhaseMachine.hpp`](src/phases/PhaseMachine.hpp)), exactly like the arm-time/launch-point carry between the normal flight phases.

#### [`CalibratePhase.hpp`](src/phases/CalibratePhase.hpp)
**ELI5:** ground maintenance — spin the drone around to teach the compass.
Motors stay off. Collects compass min/max while you rotate the drone through all orientations, then saves the calibration and returns to PARKED. Presented in the file as the template for future calibrations.

#### [`ManualPhase.hpp`](src/phases/ManualPhase.hpp)
**ELI5:** you fly it with the sticks — same machine, but the sticks set the targets.
Instead of GPS math setting the targets, the RC sticks do: throttle → climb/descend, roll/pitch → lean, yaw → turn. It reuses the exact same PID + motor mix as every other flying phase. It also has **GPS position hold**: when you center the sticks it drops an "anchor" and leans back toward it to cancel drift (reusing the nav PIDs), controllable via `MANUAL_POSITION_HOLD`. Entered/left with the radio's MANUAL switch; STOP always wins. (This is the phase the keyboard cockpit UI drives.)

---

### 7.5 `services/` — the "how"

#### [`PID.hpp`](src/services/PID.hpp)
**ELI5:** the "aim without overshooting" formula — the heart of all stability.

A PID controller answers "target is X, actual is Y — how hard do I push?" using three parts added together:
- **P** (proportional): push proportional to how far off you are *now*.
- **I** (integral): if you've been *slightly* off for a long time, push a bit harder to close the gap (this is what supplies steady hover throttle against gravity).
- **D** (derivative): if you're closing the gap *fast*, ease off so you don't overshoot.

`compute(target, actual, dt)` returns the push, clamped to a min/max. `reset()` clears its memory between flights. Everything that "holds" a value — altitude, roll, pitch, yaw, position — is a PID.

#### [`MotorController.hpp`](src/services/MotorController.hpp)
**ELI5:** the brain that turns "what I want" into "how much each of the 4 motors spins."

Owns six PIDs (altitude, roll, pitch, yaw, and two for GPS navigation). Its main method, `computeMotorMix(...)`, takes the targets + current sensor readings and produces a `MotorMix`. The **mixing** is the clever bit: to climb, add throttle to all four; to roll, add to one side and subtract from the other; to yaw, speed up the diagonal pair. Those `m1..m4 = base ± pitch ± roll ± yaw` lines are how a quadcopter steers. It also exposes `northNavigationCorrection` / `eastNavigationCorrection` used by MISSION, RTL, and MANUAL's position hold.

#### [`Motors.hpp`](src/services/Motors.hpp)
**ELI5:** the hands — the only code allowed to touch the 4 motors.

Wraps four `EspESC` drivers. `writeMix()` sends the four throttles; `disarmAll()` cuts them. In sim, every method is a no-op (no motor hardware). Nothing else in the codebase talks to motor pins — that's what keeps motor control in one auditable place.

#### [`NavMath.hpp`](src/services/NavMath.hpp)
**ELI5:** the map math — "how far, which way, and how does that split into north/east?"

Pure geometry, no state: `gpsDistanceMeters` (haversine distance), `gpsBearing` (which compass direction to fly), `bearingToNorthEast` (split a heading+distance into north and east meters), and `getMissionWaypoint` (fetch a waypoint, or hover over launch past the end). Used by MISSION, RTL, and MANUAL position hold.

#### [`Failsafes.hpp`](src/services/Failsafes.hpp)
**ELI5:** the safety net that every flying phase checks constantly.

`checkCoreFailsafes(...)`, three checks in priority order: **max flight time** → force RTL; **geofence breach** (flew too far from launch) → force RTL; **GPS lost too long** → abort straight to LANDING (can't fly home blind). Called from HOLD, MISSION, and RTL's nav ticks. (Note: MANUAL deliberately does *not* run these — manual is manual, STOP is the safety.)

#### [`RcInput.hpp`](src/services/RcInput.hpp)
**ELI5:** the radio receiver — reads your transmitter's sticks and switches.

Two layers: the **intent handlers** (`crsfHandleStart`, `crsfHandleStop`, `crsfHandleManualOn/Off`) that decide what a switch flip *means* given the current phase; and (real hardware only) `crsfTask`, which parses the raw CRSF protocol frames off the wire, latches the four sticks into `shared.sticks`, and edge-detects the switches. The handlers are shared between real radio and the simulator, so the sim tests the exact same intent logic. Also has `crsfNorm` to turn raw radio numbers into clean −1…1 values.

#### [`SimAdapter.hpp`](src/services/SimAdapter.hpp)
**ELI5:** the simulator's translator — turns little USB text commands into fake sensor data and actions. Sim builds only.

`parseSimInput()` reads text lines over USB and acts on them: `HDG:/LAT:/LON:/FIX:/ALT:` inject fake sensor readings; `CRSFSTART/STOP/MANUAL` and `STICKS:` fake the radio; `ALLOW:` approves a gated phase change; `STATUS?/MOTOR?/MANUAL?` report machine-readable state back. This is the bridge the Python test harness *and* the keyboard cockpit UI speak to. (We recently added the MANUAL/STICKS commands and the `MANUAL?` query here.)

#### [`Log.hpp`](src/services/Log.hpp)
**ELI5:** safe printing (so two cores don't scramble each other's messages) + a "halt on fatal bug" macro.

`logLine()` holds a mutex while printing so log lines never interleave. `PANIC(msg)` prints once and freezes the chip forever — used for unrecoverable setup bugs so a broken drone never flies. **Note:** in sim, these exact log strings *are* the test protocol, so their wording matters.

#### [`Imu.hpp`](src/services/Imu.hpp)
**ELI5:** the inner-ear — turns raw accel/gyro into "how am I tilted?"

Owns the MPU6050 chip and a **Madgwick filter** that fuses accelerometer + gyroscope into stable roll/pitch/yaw angles. Read every physics tick on real hardware. In sim it's left at zero (attitude control isn't exercised in the HIL tests). *Quirk:* the fused angles are stored in fields named `gyroX/Y/Z` — they're actually roll/pitch/yaw, not raw gyro.

#### [`Gps.hpp`](src/services/Gps.hpp)
**ELI5:** the "where am I on Earth" receiver.
Owns a TinyGPSPlus parser on a serial port; `read()` returns a fresh fix (lat/lon/sats/speed) when one is valid and recent, else reports no fix. Real builds only; in sim the position is injected.

#### [`Compass.hpp`](src/services/Compass.hpp)
**ELI5:** the "which way am I facing" sensor, plus its calibration ritual.
Owns the QMC5883L magnetometer. `readHeadingDeg()` gives the live heading. The calibration methods (start/sample/finish, driven by [`CalibratePhase`](#calibratephasehpp)) work out the hard-iron/soft-iron corrections and save them to flash so you only calibrate once per environment.

#### [`Altimeter.hpp`](src/services/Altimeter.hpp)
**ELI5:** the "how high above where I took off" sensor.
Wraps the barometer ([`EspBarometer`](#espbarometerhpp)). On `begin()` it averages ~2 seconds of readings to lock in the ground level, so `readAltitudeFt()` reports height *above that spot* (not sea level). Converts meters → feet.

---

### 7.6 `hardware/` — the lowest level

The bottom of the stack: code that talks to specific chips and pins.

#### [`IESC.hpp`](src/hardware/IESC.hpp)
**ELI5:** the idea of "an ESC" as an interface, with the motor layout diagram.
Defines what any ESC driver *should* offer (initialize/write/disarm/getMotor) and documents the physical motor numbering (M1 front-left … M4 rear-right, with spin directions). **See the note in [section 11](#11-honest-notes--rough-edges):** the real driver doesn't currently inherit from this interface.

#### [`EspESC.hpp`](src/hardware/EspESC.hpp)
**ELI5:** the actual motor driver — speaks "DShot600," the digital language ESCs understand.
One instance per motor. Uses the ESP32's **RMT** peripheral (a precise pulse generator) to send DShot600 frames: it converts a 0.0–1.0 throttle to the DShot number range, builds the 16-bit frame with a checksum, and transmits it as precisely-timed pulses. This is the most hardware-specific file in the project.

#### [`IBarometer.hpp`](src/hardware/IBarometer.hpp)
**ELI5:** the idea of "a barometer" as an interface.
Says any barometer must offer `initialize()` and `readAltitudeMeters()`. (The header text mentions BMP280, but the real driver is a BME280 — see below.)

#### [`EspBarometer.hpp`](src/hardware/EspBarometer.hpp)
**ELI5:** the actual barometer driver — reads the BME280 pressure chip and turns pressure into altitude.
Implements `IBarometer`. On init it finds the chip on I2C and averages 20 readings to set a ground-pressure reference. Used by the [`Altimeter`](#altimeterhpp) service.

---

## 8. Simulation & testing

You do **not** need a real drone to run and test this. Everything lives in the [`simulate/`](simulate/) folder.

**How it works.** You flash the special `wokwi_sim` build to an ESP32 (or run it under the Wokwi emulator). In that build, the real sensor code is compiled out and replaced by injected values over USB. A driver on your computer feeds fake sensor readings in and reads state back, exercising the *real flight logic* on the *real chip* — this is called **HIL (Hardware-In-the-Loop)** testing.

Two drivers can play that role:

1. **The Python harness** — [`simulate/hil_runner.py`](simulate/hil_runner.py) runs scripted **scenarios** from [`simulate/scenarios/`](simulate/scenarios/) (e.g. `full_flight_test.py`, `manual_flight_test.py`). Each scenario sends commands like `set_world(lat=...)` and asserts the drone reacts correctly. Run them with [`simulate/run_all_hil.sh`](simulate/run_all_hil.sh). This is your automated regression test suite.

2. **The keyboard cockpit** — [`simulate/manual_ui.html`](simulate/manual_ui.html), a single local web page you open in Chrome. It talks to the ESP over USB (Web Serial API), lets you **fly MANUAL mode with the arrow keys**, and visualizes the drone live (attitude, compass, altitude, motor %, and a top-down map). It runs a **light physics model** in the browser and feeds the simulated position/altitude back to the firmware — so the firmware's *real* controller flies against it. It's the same HIL idea as the Python runner, but interactive.

The bridge both of them speak to is [`SimAdapter.hpp`](#simadapterhpp) on the firmware side. The command/response protocol is documented at the top of that file.

> **Key insight:** the difference between the automated tests and the cockpit is only *where the fake GPS numbers come from* — a script types them in one case, a physics model generates them in the other. The firmware, the sensors-are-fake trick, and the phase logic are identical.

**What HIL does *not* cover (important):** most scenarios leave the IMU at zero, so the drone always "believes" it is level. That means the **navigation/phase brain** (which waypoint, when to come home, when to land) is tested thoroughly, but the **stabilization brain** (the roll/pitch/yaw PIDs + motor mixing that keep it upright) is not exercised by them. To partly close that gap, [`scenarios/stabilization_reaction_test.py`](simulate/scenarios/stabilization_reaction_test.py) injects a *fake tilt* (the `IMU:roll,pitch,yawRate` command) and asserts the firmware pushes the correct motors the correct way. This is **open-loop**: the injected tilt does not change in response to the motors, so it catches **sign / axis / mixing / clamp** bugs (the kind that flip a drone instantly) but **cannot** validate PID tuning or whether the corrections actually settle the drone. That — and confirming a positive tilt *number* really means the drone is physically leaning that way (IMU mounting + the Madgwick filter, which only run on hardware) — still requires a real, tethered bench test.

---

## 9. Common tasks

**I want to change how the drone behaves (heights, speeds, limits, waypoints).**
→ [`FlightConfig.hpp`](src/state/FlightConfig.hpp). Almost every tunable lives there.

**I want to add a new flight mode (phase).**
1. Add it to the *end* of the `FlightPhase` enum in [`FlightModel.hpp`](src/models/FlightModel.hpp) (+ its name in `phaseName`).
2. Add per-phase state structs in [`PhaseState.hpp`](src/state/PhaseState.hpp) and to `SharedState`.
3. Create `phases/YourPhase.hpp` implementing [`IFlightPhase`](src/phases/IFlightPhase.hpp) (copy an existing phase as a template).
4. Register it in [`PhaseRegistry.hpp`](src/phases/PhaseRegistry.hpp) — include it, add an instance, and add it to `table[]` **in enum order**.
5. Add a `transitionTo(PHASE_YOURS, ...)` somewhere that should trigger it.
([`ManualPhase`](src/phases/ManualPhase.hpp) is a recent, complete example of exactly these steps.)

**I want to tune the flight feel (wobbly, sluggish, drifty).**
→ The PID gains in the [`MotorController`](src/services/MotorController.hpp) constructor. Raise a P gain for a snappier response, add D to reduce overshoot. Test the change in the cockpit UI before flying.

**I want to change what a radio switch does.**
→ [`RcInput.hpp`](src/services/RcInput.hpp) (the `crsfHandle...` functions) and the channel map in [`FlightConfig.hpp`](src/state/FlightConfig.hpp).

**I want to understand a specific log message.**
→ Search the string; every log line is a `logLine("...")`. Remember these strings double as the sim test protocol, so don't reword them casually.

---

## 10. Glossary

- **ESC** — Electronic Speed Controller. The board that drives one motor. You send 0.0–1.0; it spins the motor.
- **DShot600** — a digital protocol for talking to ESCs (more precise than old analog PWM). Implemented in [`EspESC.hpp`](src/hardware/EspESC.hpp).
- **IMU** — Inertial Measurement Unit (accelerometer + gyroscope). Tells you tilt and rotation.
- **Madgwick filter** — the math that fuses accel + gyro into stable angles.
- **PID** — the target-vs-actual control formula. See [`PID.hpp`](src/services/PID.hpp).
- **Mutex** — the "pen"; a lock that lets only one core touch shared data at a time.
- **FreeRTOS / task** — the mini-OS that runs the parallel loops ("tasks") on the two cores.
- **Phase** — one self-contained flight behavior (PARKED, MISSION, …).
- **Failsafe** — an automatic safety reaction (return home / land) when something's wrong.
- **RTL** — Return To Launch. The come-home autopilot.
- **CRSF / ELRS** — the radio protocol / radio system your transmitter uses.
- **Geofence** — an invisible max-distance circle around the launch point.
- **HIL** — Hardware-In-the-Loop: testing real firmware on the real chip with faked sensors.
- **Waypoint** — a GPS point (lat/lon/altitude) the mission flies to.
- **`WOKWI_SIM`** — the compile-time flag that switches between real hardware and simulator builds.

---

## 11. Honest notes & rough edges

Because this code was largely AI-written, here are a few things a newcomer should know so they aren't confused:

- **The filename is misleading.** [`ESP32_MPU_6050_Web_Server.ino`](src/ESP32_MPU_6050_Web_Server.ino) has no web server. It's a stale name from an earlier version.
- **`IESC.hpp` is not actually used as an interface.** [`EspESC`](src/hardware/EspESC.hpp) does *not* inherit from [`IESC`](src/hardware/IESC.hpp), and [`Motors`](src/services/Motors.hpp) uses `EspESC` directly. The interface documents intent (and the useful motor-layout diagram) but isn't wired in polymorphically. [`IBarometer`](src/hardware/IBarometer.hpp) *is* implemented by [`EspBarometer`](src/hardware/EspBarometer.hpp), but it too is used concretely, not through the interface.
- **IMU field names are a little wrong.** In [`Imu.hpp`](src/services/Imu.hpp), the fused roll/pitch/yaw angles are stored in fields named `gyroX/gyroY/gyroZ`. They're angles, not raw gyro rates. This naming flows through the whole codebase.
- **The altitude controller has no explicit hover feedforward.** Steady hover throttle comes from the PID's integral term winding up, which is slightly slow. (We noticed this while building the cockpit sim; it's a candidate for tuning, not a bug.)
- **Phases intentionally duplicate code.** The near-identical `physicsTick()` in each phase is a deliberate choice (full isolation, so one phase can't break another), not an oversight. Resist the urge to "DRY" them into a base class unless you really mean to change that design decision.
- **The `#ifdef WOKWI_SIM` blocks matter.** When reading a file, notice which branch is the "real" one and which is the "sim" one; behavior genuinely differs (especially sensor input and the phase-change gate).

Welcome aboard — start with the [`.ino`](src/ESP32_MPU_6050_Web_Server.ino) and follow the [guided tour](#5-a-guided-tour-one-whole-flight). 🚁
