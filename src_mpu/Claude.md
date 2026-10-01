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
   - [`services/` — the "how" (motors, math, safety, radio switches)](#75-services--the-how)
   - [`hardware/` — the real drone's parts](#76-hardware--the-real-drones-parts)
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
- **C++ for a C# developer**: parts are made with `new` and kept in a variable of their interface type (`IGps* gps = new SimGps(world);`), just like C#. The `*` in `IGps*` means "a variable holding an object" (every C# class variable already works that way), and `->` is C#'s `.` on such a variable (`gps->read()` = `gps.Read()`). `: public IGps` is C#'s `: IGps`, and `override` means the same thing.
- **`.hpp` files**: this project is almost entirely **header files** (`.hpp`). For our purposes think of each as "one module, fully contained in one file." The `#pragma once` at the top just means "don't paste me in twice."

---

## 3. The big picture

Here's the one mental model that makes everything else click:

> **Think of the drone as a person doing a job, using a shared notebook.**

- **The notebook** = one big shared data structure called `shared` (in [`PhaseState.hpp`](#phasestatehpp)). Everything the drone knows — sensor readings, what it's aiming for, which mode it's in — is written in this notebook.
- **The senses** = the **plugs** in [`FlightIo.hpp`](#flightiohpp) (`IImu`, `IGps`, `ICompass`, `IAltimeter`, …). The flight controller asks each one for a reading and writes it into the notebook. What's behind a plug (a real chip, or the simulator) is the app's choice.
- **The current job** = the **phase** (PARKED, TAKEOFF, MISSION, LANDING, MANUAL, …). Only one phase is "active" at a time. The phase reads the notebook, decides what to aim for, and writes commands.
- **The hands** = the **Motors** service, which takes the phase's decision and actually spins the props.
- **The pen rule** = the **mutex**. Because two cores share the one notebook, you must "hold the pen" (`withMutex`) whenever you read or write it, so two writes never smear together.

Everything in the codebase is one of those roles. When you open a new file and wonder "what is this?", ask: *is it a sense, the notebook, a job, the hands, or the plumbing that connects them?*

```
        SENSORS (plugs)                    THE NOTEBOOK                 THE JOB (phase)         THE HANDS
   IImu / IGps / ICompass / ...     ──►   shared state   ──►    ParkedPhase / RaisePhase / ...  ──►  Motors ──► IMotors
        "what's true right now"          (guarded by a         "given that, what do I aim for      "spin the
                                          mutex 'pen')          and how hard do I push?"            props"
```

---

## 4. The five core ideas

Understand these five and you understand the architecture. Every file is an instance of one of them.

### Idea 1 — Two loops on two cores

The drone thinks at **two speeds**, set up in [`FlightController.hpp`](#flightcontrollerhpp):

| Task | How often | Job |
|---|---|---|
| **navigation task** | every 100 ms (**10 Hz**) | slow "where am I going" thinking: read GPS/compass, pick targets, run failsafes, switch phases |
| **physics task** | every 5 ms (**200 Hz**) | fast "keep me stable" thinking: run the PID + motor math and drive the motors |

Why two? Steering to a waypoint doesn't need to happen 200 times a second, but *staying upright* does. Splitting them means the fast stability loop is never slowed down by the slow navigation loop. The timing numbers live in [`FlightConstants.hpp`](src/flight/state/FlightConstants.hpp) (`NAV_LOOP_MS`, `PHYSICS_LOOP_MS`). (A third, small loop listens to the radio every 2 ms.)

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

A **service** wraps one messy real-world thing behind a clean method. The phases never talk to a GPS chip's raw bytes; they just read `shared.raw.gps.lat`, which the flight controller filled in from the GPS plug. Motors are the same: a phase computes a `MotorMix` and hands it to [`Motors`](#motorshpp) — nothing else in the codebase touches motor pins. This is why you can swap real hardware for a simulator without the flight logic noticing.

### Idea 5 — One flight controller, many apps

The flight controller ([`src/flight/`](src/flight/)) is a **library with a public API** ([`FlightController.hpp`](#flightcontrollerhpp)). It has **no settings and no test modes of its own**. Everything else is an **app** ([`src/apps/`](src/apps/)): a small program that loads the settings, picks where the sensor readings come from, and tells the flight controller what to do.

```
   data/flightsettings.json ──►  app (fly / first_mission / sim / bench_test / ...)
                                   │  fc::loadSettings()   fc::begin(io)   fc::start()  fc::land() ...
                                   ▼
                         the flight controller (src/flight)
                                   │  asks each plug for a reading, hands the motors plug its commands
                                   ▼
      the seven plugs (FlightIo.hpp): IImu  IAltimeter  IGps  ICompass  IBatterySensor  IMotors  IRadio
                                   │
               HardwareIo (the real chips)   or   the sim app's fakes (they read a simulated world)
```

- **Settings** live in [`data/flightsettings.json`](#flightsettingsjson), like `appsettings.json` in C#. The drone reads it from its flash at power-up and **refuses to run if anything is missing**. There are no hidden defaults in the code.
- **Inputs and outputs are injected** through [`FlightIo`](#flightiohpp): **one small interface per sensor**, one for the motors, one for the radio (like constructor injection in C#). The real drone plugs in the real chips' drivers (`realHardware()`); the simulator plugs in fakes that read a simulated world, and a sim scenario can wrap any one plug to break it on purpose. The flight logic in between is identical, and there is **no `SIM` compile flag** anywhere.
- **Each job is its own flashable app** (`pio run -e <app> -t upload`): `fly`, `first_mission`, `bench_test`, `esc_calibration`, `compass_calibration`, `sim`.

> **History:** this used to be one `.ino` program with a `SIM` compile flag and `*_ON_BOOT` switches for bench tests and calibrations. Before that, there was also a laptop "HIL" harness that injected sensors over USB. Both are gone: the settings file, the apps, and the `FlightIo` injection point replaced them.

---

## 5. A guided tour: one whole flight

The best way to learn the code is to follow what happens, in order, during a real flight. Each step links to the file that does the work.

1. **Power on.** The chip runs `setup()` in the [`fly` app](src/apps/fly/main.cpp). It loads [`flightsettings.json`](#flightsettingsjson) and calls `fc::begin()` in [`FlightController.hpp`](src/flight/FlightController.hpp), which creates the two "pens" (mutexes), brings up each plug (the real sensors, ESCs and radio from `HardwareIo`), and starts the nav, physics and radio tasks. The drone starts in **PARKED** (motors off) — that's the default in [`PhaseState.hpp`](src/flight/state/PhaseState.hpp).

2. **Idle in PARKED.** [`ParkedPhase`](src/flight/phases/ParkedPhase.hpp) does almost nothing except keep disarming the motors every tick (belt-and-suspenders safety) and refresh the dashboard numbers.

3. **You flip the START switch** on the radio. The radio plug hands the flight controller a frame, and [`RcInput`](src/flight/services/RcInput.hpp) sees the switch flip and calls `crsfHandleStart()`. If there's a GPS fix, it switches the drone to **RAISE** (takeoff).

4. **The switch happens** inside `transitionTo()` in [`PhaseMachine.hpp`](src/flight/phases/PhaseMachine.hpp). This is the *only* place phases change. It builds a little "entry context" (current height, heading, position) and calls the new phase's `onEnter()`.

5. **Takeoff (RAISE).** [`RaisePhase`](src/flight/phases/RaisePhase.hpp) slowly raises its *target altitude* from 0 to 15 ft. Each fast tick it calls the [`MotorController`](src/flight/services/MotorController.hpp), which runs a PID and produces four motor numbers, handed to [`Motors`](src/flight/services/Motors.hpp). When it reaches height, it switches to **HOLD**.

6. **Hover (HOLD).** [`HoldPhase`](src/flight/phases/HoldPhase.hpp) just holds altitude and waits. It also runs the **failsafes** ([`Failsafes.hpp`](src/flight/services/Failsafes.hpp)) every tick — the safety net that lands the drone if it flies too long or the battery runs low, brings it home if it flies too far or loses the radio, and lands it if it loses GPS.

7. **You flip START again** → the mission begins. [`MissionPhase`](src/flight/phases/MissionPhase.hpp) reads the route (from [`flightsettings.json`](#flightsettingsjson), or one an app set with `fc::setMission`), uses [`NavMath`](src/flight/services/NavMath.hpp) to work out "how far and which way to the next point," tilts the drone to fly there, and advances through the list.

8. **Mission done → HOVER_SETTLE → LANDING.** [`HoverSettlePhase`](src/flight/phases/HoverSettlePhase.hpp) pauses to bleed off momentum, then [`LandingPhase`](src/flight/phases/LandingPhase.hpp) walks the target altitude back down to the ground.

9. **Touchdown (LANDED).** [`LandedPhase`](src/flight/phases/LandedPhase.hpp) cuts the motors. It behaves like PARKED but means "finished a flight." START can re-arm from here.

10. **At any point: STOP switch** → instantly back to PARKED, motors cut ([`RcInput`](src/flight/services/RcInput.hpp) `crsfHandleStop()`). And if a failsafe trips mid-flight, the drone comes home on its own: [`RtlClimbPhase`](src/flight/phases/RtlClimbPhase.hpp) → [`RtlReturnPhase`](src/flight/phases/RtlReturnPhase.hpp) → [`RtlSettlePhase`](src/flight/phases/RtlSettlePhase.hpp) → LANDING (three ordinary phases, each handing off to the next).

If you read those ten files in that order, you'll have seen ~80% of the system working together.

---

## 6. The directory map

```
data/
├── flightsettings.json             ← every tunable setting (put on the drone with: pio run -e fly -t uploadfs)
└── flightsettings/                 override files layered on top by some apps (first_mission, sim, sim.<scenario>)

src/
├── flight/                         THE FLIGHT CONTROLLER (a library: no settings, no test modes)
│   ├── FlightController.hpp        ← START HERE: the public API (fc::begin, fc::start, fc::land, ...)
│   ├── FlightIo.hpp                the plugs: one small interface per sensor, + motors + radio
│   │
│   ├── state/          the drone's memory & settings
│   │   ├── FlightSettings.hpp      the settings struct + the flightsettings.json loader
│   │   ├── FlightConstants.hpp     the few true internals (the loop rates)
│   │   └── PhaseState.hpp          the shared "notebook" + the mutex "pen"
│   │
│   ├── models/         plain data shapes (no logic)
│   │   ├── FlightModel.hpp         the list of phases & transition reasons (+ their names)
│   │   ├── SensorTypes.hpp         shapes for sensor readings, the battery, & RC sticks
│   │   └── ControlTypes.hpp        the MotorMix shape (4 motor numbers)
│   │
│   ├── phases/         the flight behaviors (one file each)
│   │   ├── IFlightPhase.hpp        the 3-method contract every phase implements
│   │   ├── PhaseSwitch.hpp         forward-declaration of transitionTo() (breaks a cycle)
│   │   ├── PhaseMachine.hpp        transitionTo(): the ONE place phases change
│   │   ├── PhaseRegistry.hpp       the lookup table: phase enum → phase object
│   │   ├── ParkedPhase.hpp         on the ground, motors off
│   │   ├── RaisePhase.hpp          takeoff climb
│   │   ├── HoldPhase.hpp           hover & wait (runs failsafes)
│   │   ├── MissionPhase.hpp        fly the GPS waypoint route
│   │   ├── HoverSettlePhase.hpp    pause after the last waypoint
│   │   ├── LandingPhase.hpp        controlled descent
│   │   ├── LandedPhase.hpp         touched down, motors cut
│   │   ├── RtlClimbPhase.hpp       RTL step 1: climb to a safe altitude
│   │   ├── RtlReturnPhase.hpp      RTL step 2: fly back over launch
│   │   ├── RtlSettlePhase.hpp      RTL step 3: settle, then land
│   │   ├── CalibratePhase.hpp      ground maintenance: compass calibration
│   │   └── ManualPhase.hpp         fly by RC sticks (+ GPS position hold)
│   │
│   ├── services/       the "how": motors, math, safety, radio switches, logging
│   │   ├── PID.hpp                 the P-I-D control formula
│   │   ├── MotorController.hpp     PIDs + motor mixing (targets → 4 motor numbers)
│   │   ├── Motors.hpp              the only thing phases use to drive the motors (→ the FlightIo)
│   │   ├── Mission.hpp             the route MISSION flies
│   │   ├── NavMath.hpp             GPS geometry (distance, bearing, N/E split, forward/right)
│   │   ├── Failsafes.hpp           the safety net (timeout / geofence / GPS loss / radio loss / battery)
│   │   ├── Battery.hpp             fuel gauge: pack voltage + estimated mAh used → OK / WARNING / CRITICAL
│   │   ├── RcInput.hpp             what the radio's switches MEAN (START / STOP / MANUAL / LAND)
│   │   └── Log.hpp                 thread-safe serial printing + PANIC()
│   │
│   └── hardware/       what plugs into each socket: the real parts and the sim's fakes, mirrored
│       ├── HardwareIo.hpp          realHardware() and simHardware(): `new` one of each part → fc::begin(...)
│       ├── real/                   the real drone
│       │   ├── Mpu6050Imu.hpp          IImu: accel/gyro → attitude (MPU6050 + Madgwick)
│       │   ├── Bme280Altimeter.hpp     IAltimeter: barometer → height above ground (BME280)
│       │   ├── Bn880Gps.hpp            IGps: GPS receiver (BN-880 via TinyGPSPlus)
│       │   ├── Qmc5883Compass.hpp      ICompass: magnetometer + calibration (QMC5883L)
│       │   ├── AdcBatterySensor.hpp    IBatterySensor: pack voltage through a resistor divider
│       │   ├── PwmMotors.hpp           IMotors: the 4 ESCs
│       │   ├── CrsfRadio.hpp           IRadio: the ELRS receiver (CRSF frames → sticks + switches)
│       │   ├── EspPwmESC.hpp           helper: one ESC's PWM signal (ESP32 LEDC)
│       │   ├── EspBarometer.hpp        helper: the BME280 chip
│       │   └── I2cBus.hpp              helper: starts the I2C wires the IMU, compass and barometer share
│       └── sim/                    the simulator's fakes (used only by the sim app)
│           ├── SimImu.hpp              IImu: the world's true tilt + heading
│           ├── SimAltimeter.hpp        IAltimeter: the world's true height
│           ├── SimGps.hpp              IGps: the world's true position as lat/lon
│           ├── SimCompass.hpp          ICompass: the world's true heading
│           ├── SimBatterySensor.hpp    IBatterySensor: the world's pack voltage
│           ├── SimMotors.hpp           IMotors: hands the commands to the world
│           ├── SimRadio.hpp            IRadio: the fake transmitter the scenario's pilot holds
│           └── SimWorld.hpp            what's TRUE: QuadSim physics + the pack's charge, on its own 200 Hz clock
│
└── apps/                           THE PROGRAMS (flash one: pio run -e <app> -t upload)
    ├── fly/main.cpp                the real flight program
    ├── first_mission/main.cpp      the real flight program, with a short 30 m out-and-back mission
    ├── bench_test/main.cpp         props OFF: spin one motor at a time, stream every sensor
    ├── esc_calibration/main.cpp    props OFF: teach the ESCs the throttle range (once)
    ├── compass_calibration/main.cpp  outdoors: calibrate the compass (once)
    ├── sim/                        the on-chip simulator (see §8)
    │   ├── main.cpp                picks the scenario, plugs in simHardware(), answers DUMPLOG
    │   ├── FlightLog.hpp           the flight recorder (/flight.csv)
    │   └── scenarios/              one file per test (+ Scenario.hpp: the rig and the pilot)
    └── common/                     shared by apps: SerialInput.hpp (reads what you type)
```

> **Reading order for a newcomer:** the [`fly` app](src/apps/fly/main.cpp) → [`FlightController.hpp`](#flightcontrollerhpp) → [`flightsettings.json`](#flightsettingsjson) → [`PhaseState.hpp`](#phasestatehpp) → [`IFlightPhase.hpp`](#iflightphasehpp) → [`ParkedPhase.hpp`](#parkedphasehpp) → [`RaisePhase.hpp`](#raisephasehpp) → [`MotorController.hpp`](#motorcontrollerhpp) → [`PID.hpp`](#pidhpp). After that, the rest falls into place.

---

## 7. File-by-file reference

Every file below has: a one-line **ELI5**, what it does, and how it connects to the rest.

### 7.1 Entry point: the API, the IO, and the apps

#### [`FlightController.hpp`](src/flight/FlightController.hpp)
**ELI5:** the front door of the flight controller: the buttons an app is allowed to press.

The **public API**, and the "composition root" (like `Program.cs`): `fc::begin()` makes each service with `new` (`Motors`, `MotorController`, `Failsafes`, `Battery`), then makes each phase, handing it exactly the services it needs in its constructor. No service is a global. The only shared globals are the notebook (`shared`) and its two mutexes, born here and borrowed everywhere as `extern`, which is why an app includes this file **exactly once**, from its `main.cpp`.

What an app can call:
- **Setup:** `fc::loadSettings(errors, {overrides...})` (reads [`flightsettings.json`](#flightsettingsjson), then any override files on top), `fc::begin(io)` (plug in the sensors, motors and radio, bring each up, start the loops), `fc::halt(why)`. Settings are only ever changed through files, never from code.
- **Commands** — the buttons; the radio presses these when you flip a switch, and an app can too: `fc::start()`, `fc::stop()`, `fc::land()`, `fc::manualOn()` / `manualOff()`, `fc::setMission(route)`, `fc::calibrateCompass()`. (Stick positions only ever come from the radio plug.)
- **Read-only state:** `fc::phase()`, `fc::sensors()`, `fc::batteryState()`, `fc::lastMotorMix()`, `fc::missionWaypointIndex()`, `fc::launchPoint(...)`.

Inside, it runs three loops pinned to the two cores (`xTaskCreatePinnedToCore`):
- **navigation task** (10 Hz): read the GPS and compass plugs, write them into `shared`, run the current phase's `navTick()`.
- **physics task** (200 Hz): read the IMU, altimeter and battery-sensor plugs, run the current phase's `physicsTick()`, then update the battery fuel gauge.
- **radio task** (every 2 ms): ask the radio plug for a new frame; if there is one, hand it to [`RcInput`](#rcinputhpp) (sticks, switch flips, "the radio is alive").

#### [`FlightIo.hpp`](src/flight/FlightIo.hpp)
**ELI5:** the sockets the flight controller's senses, hands and ears plug into: one socket per sensor.

Seven small interfaces (like C# `interface`s), each with an optional `begin()`:

| Plug | Gives the flight controller | Real drone ([`hardware/`](src/flight/hardware/)) |
|---|---|---|
| `IImu` | roll / pitch / yaw + yaw rate | `Mpu6050Imu` |
| `IAltimeter` | height above launch, ft | `Bme280Altimeter` |
| `IGps` | lat / lon / fix | `Bn880Gps` |
| `ICompass` | heading (+ calibration) | `Qmc5883Compass` |
| `IBatterySensor` | pack volts | `AdcBatterySensor` |
| `IMotors` | ← takes the 4 motor commands | `PwmMotors` |
| `IRadio` | radio frames: sticks + switch positions | `CrsfRadio` |

`FlightIo` is just the bundle of those seven pointers, handed to `fc::begin()`. The radio is the only optional one: with no radio there's no pilot, so nothing ever tells the drone to take off (that's how `compass_calibration` stays safely on the ground). **[`realHardware()`](src/flight/hardware/HardwareIo.hpp)** makes one of each real driver with `new` and returns them bundled: `fc::begin(realHardware())`. Small apps can use a part directly (`hardware.motors->writeOne(...)`). The sim app plugs in `simHardware()` from the same file instead (the fakes in `hardware/sim/`, see §8). The motor layout diagram (which corner is M1, which way each prop spins) lives on `IMotors`.

#### The apps ([`src/apps/`](src/apps/))
Each is a small `main.cpp` with its own `setup()`/`loop()`, flashed with `pio run -e <app> -t upload`:

| App | What it does |
|---|---|
| [`fly`](src/apps/fly/main.cpp) | the real flight program: settings → `fc::begin(realHardware())` → the radio drives it |
| [`first_mission`](src/apps/first_mission/main.cpp) | the same, but it loads [`flightsettings/first_mission.json`](data/flightsettings/first_mission.json) on top: the mission becomes a 30 m out-and-back along the first leg of the route |
| [`bench_test`](src/apps/bench_test/main.cpp) | props off: `1`–`4` spins one motor, `s` streams every sensor. Never starts the flight loops or radio. |
| [`esc_calibration`](src/apps/esc_calibration/main.cpp) | props off: the one-time GO / MIN throttle-range routine, using only the ESCs |
| [`compass_calibration`](src/apps/compass_calibration/main.cpp) | the one-time compass calibration (the `CALIBRATE` phase), with no radio so it can't take off |
| [`sim`](src/apps/sim/main.cpp) | the on-chip simulator (§8) |

---

### 7.2 `state/`: settings & the shared notebook

#### [`flightsettings.json`](data/flightsettings.json)
**ELI5:** the settings menu: every number you might want to tweak, in one file, like `appsettings.json` in C#.

Sections: `airframe` (hover throttle and all the PID gains), `flight` (takeoff height, climb/descent speeds, waypoint radius, line-following lookahead), `safety` (max flight time, geofence, GPS/radio-loss timings, RTL height), `battery` (pack size and the fuel-gauge thresholds), `manual` (stick feel), `radio` (which channel is which stick or switch, numbered CH1–CH16 like the radio's screen, and how far up a switch counts as "up"), `wiring` (every pin, the ESC pulse rate), `calibration`, and `mission` (the route). If you want to change *how the drone behaves* without changing *how it works*, you come here. After editing, put it on the drone with `pio run -e fly -t uploadfs`.

**Override files** work like `appsettings.Development.json` in .NET: an app can layer extra files on top, each holding only what it changes. Sections merge key by key; a list (like the route) is replaced whole. Unlike .NET, an override an app asks for is **required**, so a missing one stops the app instead of quietly flying the base settings.

| File | Loaded by | What it changes |
|---|---|---|
| `flightsettings/first_mission.json` | `first_mission` app (and the sim's `testroute` scenario) | the mission: a 30 m out-and-back at 20 ft |
| `flightsettings/sim.<scenario>.json` | `sim` app — every scenario has one (required) | `sim.geofence`: a 40 m fence; `sim.timeout`: an 18 s limit; the rest are `{}` (no changes) |

#### [`FlightSettings.hpp`](src/flight/state/FlightSettings.hpp)
**ELI5:** the reader for that file, and the strict inspector.

Defines the `FlightSettings` struct (one field per setting) and `loadFlightSettings()`, which reads the JSON with ArduinoJson. **Every** setting is required: if one is missing or the wrong type, it lists every problem ("battery.capacityMah is missing") and the app refuses to run. Code reads the settings through `settings()`. The few true internals that aren't settings (the loop rates) live in [`FlightConstants.hpp`](src/flight/state/FlightConstants.hpp).

#### [`PhaseState.hpp`](src/flight/state/PhaseState.hpp)
**ELI5:** the shared notebook, and the pen rule for writing in it.

The single most important state file. It defines:
- Three little structs **per phase**: `Dashboard_X` (numbers to show), `Cruise_X` (what the phase is aiming for), `Trip_X` (private working memory). The header comment has a great rule of thumb for which to use.
- `SharedState` — the whole notebook, bundling every phase's blocks plus `raw` (latest sensor readings), the current `phase`, and the RC `sticks`.
- `withMutex(fn)` — "grab the pen, run your code, put the pen back." **Always** used when touching `shared`.

The many `operator=(const volatile ...&)` bits are a C++ detail: `shared` is marked `volatile` (it changes across cores), and these let you copy a block out of it cleanly.

---

### 7.3 `models/` — plain data shapes

These three files contain zero logic — just the "shapes" of data that flow around.

#### [`FlightModel.hpp`](src/flight/models/FlightModel.hpp)
**ELI5:** the master list of every flight mode and every reason it might switch.

Defines the `FlightPhase` enum (PARKED, RAISE, HOLD, MISSION, RTL_CLIMB, RTL_RETURN, RTL_SETTLE, HOVER_SETTLE, LANDING, LANDED, CALIBRATE, MANUAL), the `TransitionReason` enum (why a switch happened, for logging), and helper functions that turn each into a readable string. Add new phases at the *end* of the enum (the registry's table is sized by the last one — see [`PhaseRegistry.hpp`](#phaseregistryhpp)).

#### [`SensorTypes.hpp`](src/flight/models/SensorTypes.hpp)
**ELI5:** the shapes for "a GPS reading," "an IMU reading," and "the RC sticks."

Structs: `RawImuReading` (tilt/spin/accel), `RawGpsReading` (lat/lon/fix/sats), `RawSticks` (throttle/roll/pitch/yaw from the radio), and `RawSensors` which bundles them together. These are what the services fill in and the phases read.

#### [`ControlTypes.hpp`](src/flight/models/ControlTypes.hpp)
**ELI5:** the shape of a motor command — four numbers plus a few extras for display.

Just `MotorMix`: `m1, m2, m3, m4` (the four motor throttles, 0–1) plus `baseThrottle`, `rollCorrection`, `pitchCorrection` (kept for the dashboard so you can see *why* the motors are doing what they're doing). Produced by [`MotorController`](#motorcontrollerhpp), consumed by [`Motors`](#motorshpp).

---

### 7.4 `phases/` — the flight behaviors

First the four "framework" files, then the ten actual behaviors.

#### [`IFlightPhase.hpp`](src/flight/phases/IFlightPhase.hpp)
**ELI5:** the plug shape every flight mode must fit into.

The interface (like a C# `interface`). A phase implements `id()`, and any of `onEnter()`, `navTick()`, `physicsTick()` (unused ones default to doing nothing). Also defines `EnterContext` — the little bundle of "where we are right now" that `transitionTo()` fills in and hands to a phase's `onEnter()`. The header comment is worth reading; it explains the deliberate no-shared-behavior design.

#### [`PhaseSwitch.hpp`](src/flight/phases/PhaseSwitch.hpp)
**ELI5:** a one-line promise that "a function called `transitionTo` exists somewhere."

The only forward-declaration in the project. It exists to break a chicken-and-egg cycle: phases call `transitionTo()`, but `transitionTo()` calls back into phases. A phase includes this tiny header to learn the function's *shape* without pulling in its whole body. The real body is in `PhaseMachine.hpp`.

#### [`PhaseMachine.hpp`](src/flight/phases/PhaseMachine.hpp)
**ELI5:** the gearbox — the single place where the drone shifts from one mode to another.

Contains the real `transitionTo(next, reason)`. It builds the `EnterContext`, carries forward "trip" info like arm-time and launch point, calls the new phase's `onEnter()`, and records the new phase. Transitions are immediate — the same on a real drone and in the sim. If you ever wonder "how does the drone actually change modes?" — it's here, and *only* here.

#### [`PhaseRegistry.hpp`](src/flight/phases/PhaseRegistry.hpp)
**ELI5:** the phone book — given a phase name, hand back the object that runs it.

`buildPhases(...)` makes one of each phase with `new`, handing each its services (`table[PHASE_HOLD] = new HoldPhase(motors, motorController, failsafes);`), and `phaseFor(phase)` hands back the one for a given phase. Each phase has its own named slot, so the order of the lines doesn't matter.

Now the ten behaviors. They share a **common rhythm** (learn it once, and every phase reads the same): `onEnter()` sets the initial targets; `navTick()` updates the dashboard and decides whether to switch phases; `physicsTick()` copies the targets + latest sensors, calls `motorController_->computeMotorMix(...)`, sends the result to `motors_->writeMix(...)`, and records the mix for display. Each phase's **constructor** lists the services it uses (`HoldPhase(Motors* motors, MotorController* motorController, Failsafes* failsafes)`), so the top of the file tells you what it depends on.

#### [`ParkedPhase.hpp`](src/flight/phases/ParkedPhase.hpp)
**ELI5:** sitting on the ground, motors dead, waiting for START.
The safe resting state and the emergency-stop destination. Its `physicsTick()` calls `motors_->disarmAll()` *every tick* so a stop never relies on a stale command.

#### [`RaisePhase.hpp`](src/flight/phases/RaisePhase.hpp)
**ELI5:** takeoff — smoothly raise the target height, then hand off to HOLD.
Ramps `targetAltFt` from 0 up to `TAKEOFF_ALTITUDE_FT` at a fixed climb rate, keeping attitude level. Records the arm-time and launch position (needed later for RTL). Switches to HOLD when it reaches height.

#### [`HoldPhase.hpp`](src/flight/phases/HoldPhase.hpp)
**ELI5:** hover in place and wait for the next command.
Holds altitude and heading. Crucially, this is where the core **failsafes** run each tick (`failsafes_->checkCore(...)`). START from HOLD begins the mission.

#### [`MissionPhase.hpp`](src/flight/phases/MissionPhase.hpp)
**ELI5:** fly the pre-set GPS route, point to point.
Each nav tick: run failsafes, compute distance + bearing to the current waypoint using [`NavMath`](#navmathhpp), turn to face it, tilt to move toward it (the tilt amounts come from the nav PIDs), and advance to the next waypoint when close enough. After the last one → HOVER_SETTLE.

#### [`HoverSettlePhase.hpp`](src/flight/phases/HoverSettlePhase.hpp)
**ELI5:** pause and steady up after the last waypoint before landing.
Hovers for a fixed time (`MISSION_COMPLETE_HOVER_MS`) to bleed off momentum, then → LANDING.

#### [`LandingPhase.hpp`](src/flight/phases/LandingPhase.hpp)
**ELI5:** come down gently, then cut the motors.
Walks `targetAltFt` down at `LAND_DESCENT_RATE_FPS` until it reaches 0, then → LANDED. Also the direct abort target when GPS is lost (can't RTL without GPS).

#### [`LandedPhase.hpp`](src/flight/phases/LandedPhase.hpp)
**ELI5:** touched down — like PARKED, but "we just finished flying."
Motors cut. START can re-arm from here.

#### The RTL trio: [`RtlClimbPhase.hpp`](src/flight/phases/RtlClimbPhase.hpp) · [`RtlReturnPhase.hpp`](src/flight/phases/RtlReturnPhase.hpp) · [`RtlSettlePhase.hpp`](src/flight/phases/RtlSettlePhase.hpp)
**ELI5:** the "uh-oh, come home" autopilot — three ordinary phases, run back-to-back.
A failsafe (geofence, or radio loss) drops the drone into **RTL_CLIMB** (rise to `RTL_ALTITUDE_FT`), which hands off to **RTL_RETURN** (fly back over the launch point using the same GPS nav as MISSION), which hands off to **RTL_SETTLE** (hover for `RTL_SETTLE_MS` to bleed off momentum) → LANDING. Each is a normal, self-contained, individually-triggerable phase with its own `Dashboard_/Cruise_/Trip_` block — **not** a "phase inside a phase." The launch point is carried forward from one step to the next by `transitionTo()` (see the `case PHASE_RTL_CLIMB` / `case PHASE_RTL_RETURN` blocks in [`PhaseMachine.hpp`](src/flight/phases/PhaseMachine.hpp)), exactly like the arm-time/launch-point carry between the normal flight phases.

#### [`CalibratePhase.hpp`](src/flight/phases/CalibratePhase.hpp)
**ELI5:** ground maintenance — spin the drone around to teach the compass.
Motors stay off. Collects compass min/max while you rotate the drone through all orientations, then saves the calibration and returns to PARKED. Presented in the file as the template for future calibrations.

#### [`ManualPhase.hpp`](src/flight/phases/ManualPhase.hpp)
**ELI5:** you fly it with the sticks — same machine, but the sticks set the targets.
Instead of GPS math setting the targets, the RC sticks do: throttle → climb/descend, roll/pitch → lean, yaw → turn. It reuses the exact same PID + motor mix as every other flying phase. It also has **GPS position hold**: when you center the sticks it drops an "anchor" and leans back toward it to cancel drift (reusing the nav PIDs), controllable via `MANUAL_POSITION_HOLD`. Entered/left with the radio's MANUAL switch; STOP always wins.

---

### 7.5 `services/` — the "how"

#### [`PID.hpp`](src/flight/services/PID.hpp)
**ELI5:** the "aim without overshooting" formula — the heart of all stability.

A PID controller answers "target is X, actual is Y — how hard do I push?" using three parts added together:
- **P** (proportional): push proportional to how far off you are *now*.
- **I** (integral): if you've been *slightly* off for a long time, push a bit harder to close the gap (this is what supplies steady hover throttle against gravity).
- **D** (derivative): if you're closing the gap *fast*, ease off so you don't overshoot.

`compute(target, actual, dt)` returns the push, clamped to a min/max. `reset()` clears its memory between flights. Everything that "holds" a value — altitude, roll, pitch, yaw, position — is a PID.

#### [`MotorController.hpp`](src/flight/services/MotorController.hpp)
**ELI5:** the brain that turns "what I want" into "how much each of the 4 motors spins."

Owns six PIDs (altitude, roll, pitch, yaw, and two for GPS navigation). Its main method, `computeMotorMix(...)`, takes the targets + current sensor readings and produces a `MotorMix`. The **mixing** is the clever bit: to climb, add throttle to all four; to roll, add to one side and subtract from the other; to yaw, speed up the diagonal pair. Those `m1..m4 = base ± pitch ± roll ± yaw` lines are how a quadcopter steers. It also exposes `forwardNavigationCorrection` / `rightNavigationCorrection` used by MISSION, RTL, and MANUAL's position hold — fed the GPS error *after* it's rotated from north/east into the drone's own forward/right by `northEastToForwardRight` ([`NavMath.hpp`](#navmathhpp)). Yaw damping uses the real turn rate `yawRateDps`, and the total yaw push is capped at ±0.2 so yaw can never starve roll/pitch.

#### [`Motors.hpp`](src/flight/services/Motors.hpp)
**ELI5:** the hands — the only code allowed to touch the 4 motors.

Forwards to the `IMotors` plug ([`FlightIo.hpp`](#flightiohpp): the real ESCs, or the sim's physics). `writeMix()` sends the four throttles and remembers them as `lastMix` (the battery estimate uses it); `disarmAll()` cuts them. No phase touches the motors plug directly — that's what keeps motor control in one auditable place. (The one-time ESC range calibration is its own app, [`esc_calibration`](src/apps/esc_calibration/main.cpp).)

#### [`NavMath.hpp`](src/flight/services/NavMath.hpp)
**ELI5:** the map math — "how far, which way, and how does that split into north/east?"

Pure geometry, no state: `gpsDistanceMeters` (haversine distance), `gpsBearing` (which compass direction to fly), `bearingToNorthEast` (split a heading+distance into north and east meters), `northEastToForwardRight` (rotate a north/east error into the drone's own forward/right using its compass heading), and `getMissionWaypoint` (fetch a waypoint, or hover over launch past the end). Used by MISSION, RTL, and MANUAL position hold.

#### [`Failsafes.hpp`](src/flight/services/Failsafes.hpp)
**ELI5:** the safety net that every flying phase checks constantly.

A class a phase gets in its constructor. `checkCore(...)`, three checks in priority order: **max flight time** → **land where it is** (on a big field, flying home could cost more battery than is left); **geofence breach** (flew too far from launch) → force RTL; **GPS lost too long** → abort straight to LANDING (can't fly home blind). Called from HOLD and MISSION. (Note: MANUAL deliberately does *not* run these — manual is manual, STOP is the safety.)

Plus `checkRadio()`: if no radio frame arrives for `safety.radioLossTimeoutMs` (1 s), the pilot's STOP/MANUAL switches can't reach the drone, so it comes home by itself: RTL, or LANDING if it's below `safety.radioLossLandBelowFt` or has no GPS. Called from RAISE, HOLD, MISSION **and** MANUAL (with no radio there is no pilot, even in manual). It only arms once a radio has actually been heard (so the radio-less `compass_calibration` app never trips it). The sim always has a fake radio, so it's armed in every sim scenario, and `rcloss` cuts it.

Plus `checkBattery()`: as soon as the [`Battery`](#batteryhpp) service reports **WARNING or CRITICAL**, the drone **lands where it is**, never flying home first. Called from every airborne phase (RAISE, HOLD, MISSION, MANUAL, HOVER_SETTLE, and the three RTL phases), so it also cuts a return-home short if the battery runs low on the way.

#### [`Battery.hpp`](src/flight/services/Battery.hpp)
**ELI5:** the fuel gauge, modeled on INAV's battery code.
Three independent gauges, because none is trustworthy alone: (1) **pack voltage**, read on GPIO 1 through a two-resistor divider and smoothed with a 1 Hz filter so throttle punches don't trip it; (2) **estimated mAh used**, from INAV's "virtual current sensor" idea: `amps ≈ CURRENT_IDLE_A + CURRENT_MOTOR_FULL_A × (m1³ + m2³ + m3³ + m4³)` (prop power grows with speed cubed), added up over time; and (3) the **flight-time limit** in `Failsafes.hpp`. The voltage and mAh gauges each give OK / WARNING / CRITICAL (with a small buffer so it doesn't flicker), and the worse one wins. For LiFePO4, whose voltage is flat until the end, the mAh estimate is the main gauge and voltage is the backup. The START switch also refuses to take off on a low pack. All thresholds live in the `battery` section of [`flightsettings.json`](#flightsettingsjson). The voltage comes from the `IBatterySensor` plug; if the divider isn't wired, the voltage gauge is skipped and the other two still work. In the sim, [`SimWorld`](src/flight/hardware/sim/SimWorld.hpp) has a fake LiFe pack that drains, sags, and has the voltage "cliff".

#### [`RcInput.hpp`](src/flight/services/RcInput.hpp)
**ELI5:** what your transmitter's switches *mean*.

The radio itself is a plug (`IRadio`): [`CrsfRadio`](src/flight/hardware/real/CrsfRadio.hpp) on the real drone reads the receiver (through the AlfredoCRSF library), the sim's `SimRadio` is a fake transmitter. Either way, each frame says where the sticks and switches are. The flight controller's radio loop ([`FlightController.hpp`](src/flight/FlightController.hpp)) latches the sticks, marks the link alive (for the radio-loss failsafe), and presses the matching **button** for any switch that just flipped: `fc::start()`, `fc::stop()`, `fc::land()`, `fc::manualOn()` / `manualOff()` (STOP is pressed every frame it's on; the others on the flip). This file holds the **rules** behind each button (`crsfHandleStart`, …): is it allowed right now, and which phase comes next.

#### [`Log.hpp`](src/flight/services/Log.hpp)
**ELI5:** safe printing (so two cores don't scramble each other's messages) + a "halt on fatal bug" macro.

`logLine()` holds a mutex while printing so log lines never interleave. `PANIC(msg)` prints once and freezes the chip forever — used for unrecoverable setup bugs so a broken drone never flies. **Note:** in sim, these exact log strings *are* the test protocol, so their wording matters.

---

### 7.6 `hardware/` — the real drone's parts

One class per plug (see [`FlightIo.hpp`](#flightiohpp)), in two mirrored folders: `real/` (the chips below, plus their helpers) and `sim/` (the simulator's fakes, see §8). [`HardwareIo.hpp`](src/flight/hardware/HardwareIo.hpp) has `realHardware()` and `simHardware()`, which `new` one of each. Nothing outside this folder knows which chips the drone has.

#### [`HardwareIo.hpp`](src/flight/hardware/HardwareIo.hpp)
**ELI5:** the box of real parts. `realHardware()` makes one of each driver below (`io.gps = new Bn880Gps();` …) and hands back all seven, ready for `fc::begin()`.

#### [`Mpu6050Imu.hpp`](src/flight/hardware/real/Mpu6050Imu.hpp) — `IImu`
**ELI5:** the inner ear — turns raw accel/gyro into "how am I tilted?"
Owns the MPU6050 chip and a **Madgwick filter** that fuses accelerometer + gyroscope into stable roll/pitch/yaw angles. *Quirk:* the fused angles are stored in fields named `gyroX/Y/Z` — they're actually roll/pitch/yaw, not raw gyro.

#### [`Bme280Altimeter.hpp`](src/flight/hardware/real/Bme280Altimeter.hpp) — `IAltimeter`
**ELI5:** the "how high above where I took off" sensor.
Wraps the barometer chip ([`EspBarometer`](src/flight/hardware/real/EspBarometer.hpp)). On `begin()` it averages ~2 seconds of readings to lock in the ground level, so `readFt()` reports height *above that spot* (not sea level).

#### [`Bn880Gps.hpp`](src/flight/hardware/real/Bn880Gps.hpp) — `IGps`
**ELI5:** the "where am I on Earth" receiver.
A TinyGPSPlus parser on a serial port; `read()` returns a fresh fix (lat/lon/sats/speed) when one is valid and recent, else `fix = false`.

#### [`Qmc5883Compass.hpp`](src/flight/hardware/real/Qmc5883Compass.hpp) — `ICompass`
**ELI5:** the "which way am I facing" sensor, plus its calibration ritual.
`readHeadingDeg()` gives the live heading. The calibration methods (start/sample/finish, driven by [`CalibratePhase`](#calibratephasehpp)) work out the hard-iron/soft-iron corrections and save them to flash so you only calibrate once per environment.

#### [`AdcBatterySensor.hpp`](src/flight/hardware/real/AdcBatterySensor.hpp) — `IBatterySensor`
**ELI5:** the fuel-gauge needle. Reads the pack voltage on an ESP32 pin through a two-resistor divider.

#### [`PwmMotors.hpp`](src/flight/hardware/real/PwmMotors.hpp) — `IMotors`
**ELI5:** the four ESCs. One [`EspPwmESC`](src/flight/hardware/real/EspPwmESC.hpp) per motor, which sends standard PWM: 1000 µs = stopped, 2000 µs = full, 400 times a second, generated in hardware by the ESP32's **LEDC** peripheral. PWM ESCs need a one-time range calibration (the `esc_calibration` app). (An older DShot600 driver was removed: budget ESCs usually don't support DShot, and PWM works with every ESC.)

#### [`CrsfRadio.hpp`](src/flight/hardware/real/CrsfRadio.hpp) — `IRadio`
**ELI5:** the radio receiver. The **AlfredoCRSF** library (in `platformio.ini`, like a NuGet package) reads the ELRS receiver's CRSF postcards and checks none are garbled; this file just asks it "where's channel 3?" and gets microseconds (~1000 down, 1500 center, ~2000 up). Which channel is which comes from `radio` in the settings. STOP is engaged unless its switch is *up*, so an unset or middle switch is safe. When no good postcard has arrived for 300 ms, the library calls the link down and `read()` returns false (the radio-loss failsafe then lands or brings the drone home). What a flip *means* is decided in [`RcInput`](#rcinputhpp).

#### [`I2cBus.hpp`](src/flight/hardware/real/I2cBus.hpp)
The IMU, compass and barometer share two I2C wires; each driver calls `startI2c()` and only the first call does anything.

---

## 8. Simulation & testing

You do **not** need a real drone to run and test this. The whole simulation runs **on the ESP itself**.

**How it works.** The simulator is just another app: [`sim`](src/apps/sim/main.cpp). It plugs **fakes** into the same seven sockets the real chips use ([`FlightIo.hpp`](#flightiohpp)), so the flight controller can't tell the difference. So the **real controller flies against real physics, at the real 200 Hz, with no laptop in the loop.** The pieces, each in its own file in [`src/apps/sim/`](src/apps/sim/):

- 🌍 **[`SimWorld`](src/flight/hardware/sim/SimWorld.hpp)** — what's **true**: where the drone really is, how it's really tilted, how much charge the pack really has. The **QuadSim** physics library ([`lib/QuadSim/`](lib/QuadSim/)) does the work: motor thrust → force → acceleration → velocity → position. It runs on **its own 200 Hz clock**, like the real world: the drone's motors push it and its sensors look at it, but it carries on regardless.
- 👀 **[the fake parts](src/flight/hardware/sim/)** — the fake sensors. Each just looks at the world: `SimGps` turns the true position (meters from home) into lat/lon, `SimAltimeter` reports the true height, and so on. `SimMotors` hands the motor commands to the world.
- 🎮 **[`SimRadio`](src/flight/hardware/sim/SimRadio.hpp)** — a fake transmitter. The scenario's pilot flips its switches and moves its sticks; the flight controller gets frames exactly like the real receiver's, through the same switch logic.
- 🧪 **[`scenarios/`](src/apps/sim/scenarios/)** — one file per test.
- 📼 **[`FlightLog`](src/apps/sim/FlightLog.hpp)** — writes the true position, tilt and phase to `/flight.csv` 10× a second; `DUMPLOG` streams it back.

```
   flight controller ──► SimMotors ──► SimWorld (QuadSim: position, tilt, pack charge)   ◄── scenario: gust, weak pack
          ▲                                  │  its own 200 Hz clock
          │                                  ▼
          └──── SimImu / SimGps / SimCompass / SimAltimeter / SimBatterySensor         ◄── scenario: wrap one to break it
          └──── SimRadio (sticks + switches)                                            ◄── scenario: the pilot
```

**What a scenario can do** — only three things (see [`Scenario.hpp`](src/apps/sim/scenarios/Scenario.hpp)):

| Kind | How | Examples |
|---|---|---|
| change the **world** (something really happens) | `rig.world->gust(...)`, `rig.world->setCharge(...)` | `stab` (a gust), `lowbatt` (a weak pack) |
| break a **sensor** (the drone perceives it wrong) | wrap one plug in `setup()`: `rig.io.gps = new LosableGps(rig.io.gps)` | `gpsloss` (`LosableGps`), `rcloss` (`CuttableRadio`) |
| be the **pilot** | `rig.radio->pressStart()`, `pressLand()`, `setManual()`, `setSticks()`, `setStop()` | every scenario takes off this way; `land`, `manual` |

And **rule changes are settings**, not code: every scenario has its own override file `flightsettings/sim.<name>.json`, and it's required (`sim.geofence.json` shrinks the fence, `sim.timeout.json` shortens the timer, the others are `{}` because they change no settings), and a scenario can ask for another file (`testroute` loads the real `first_mission.json`). Otherwise the sim flies under the real drone's settings, including the real 5-minute flight limit — so `field_patrol` also proves the full fence line fits in it.

**Running it.** [`simulate/run_hil.sh`](simulate/run_hil.sh) uploads the settings, flashes the `sim` app, and runs the checks in [`simulate/scenarios_esp/`](simulate/scenarios_esp/). Each check resets the ESP, sends `SCENARIO:<name>` (the sim hears it before loading the settings), waits for the flight to finish, pulls the log (`DUMPLOG`), checks it, and saves a JSON the map replay reads:

```
ESP_PORT=/dev/cu.usbmodem14101 ./simulate/run_hil.sh
```

| Check | Scenario file | What it exercises |
|---|---|---|
| `field_patrol.py` | `FlyMission` (`full`) | full flight: takeoff → whole fence line → land (also drives the Clover map) |
| `geofence_breach.py` | `FlyMission` (`geofence`) + `sim.geofence.json` | fly past a 40 m fence → geofence failsafe forces RTL |
| `gps_loss.py` | `GpsLoss` | the GPS loses its fix while hovering → lands where it is (can't RTL blind) |
| `max_timeout.py` | `FlyMission` (`timeout`) + `sim.timeout.json` | flight-time limit hit → lands where it is (no RTL) |
| `low_battery.py` | `LowBattery` | pack only 25% charged → voltage gauge → lands where it is |
| `rc_loss.py` | `RadioLoss` | radio link cut mid-mission → radio failsafe → RTL → land |
| `land_switch.py` | `LandSwitch` | LAND switch flipped while hovering → lands where it is |
| `test_route.py` | `FlyMission` (`testroute`) + `first_mission.json` | the short first-flight route: 30 m out and back, lands at home |
| `manual_flight.py` | `ManualFlight` | MANUAL mode flies the drone on the sticks against the physics |
| `stabilization.py` | `Stabilization` | a gust rolls it ~22° → the controller recovers to level |

**Adding a scenario:** a new file in `scenarios/` (copy the closest one), one line in the list at the top of the sim's [`main.cpp`](src/apps/sim/main.cpp), a `data/flightsettings/sim.<name>.json` (just `{}` if it changes no settings), and a check script in `simulate/scenarios_esp/`. You never touch the world, the fake sensors, or the flight controller.

**The frame mapping (why it flies straight).** The sign conventions between QuadSim's world and the firmware's were **locked on the laptop first**, in [`lib/QuadSim/examples/nav_check.cpp`](lib/QuadSim/examples/nav_check.cpp), which runs the firmware's exact nav+mix math against QuadSim and sweeps the signs until it converges on a waypoint. The result (roll `+`, pitch `−`, north `−x`, east `−y`) went into the sim (now [`SimWorld.hpp`](src/flight/hardware/sim/SimWorld.hpp)) and flew correctly on the first hardware run. If you ever change the mixing or QuadSim's frames, re-run `nav_check` before flashing.

**What this covers — and doesn't.** Because the physics reacts to the motors (a true **closed loop**), this exercises the **whole** system: the navigation/phase brain *and* the stabilization brain (roll/pitch/yaw PIDs + motor mixing), including altitude hold, position hold, cornering and landing. It catches wrong-sign/instability bugs and lets you tune gains against realistic dynamics. **Honest limits:** QuadSim is still a *model*, not your exact airframe (its mass/inertia/thrust are generic), and it doesn't simulate the vibration that shakes a real IMU or GPS noise. It does **not** replace a real, tethered bench test — but it's a far truer test than the old open-loop harness, and it runs at the real rate on the real chip.

> **The keyboard cockpit** ([`simulate/manual_ui.html`](simulate/manual_ui.html)) was built for the *old* laptop-HIL protocol (browser physics + serial sensor injection), which has been removed. The file is kept, but flying it against the on-chip physics would need a small rework (send stick inputs, read state back, instead of running its own physics).

---

## 9. Common tasks

**I want to change how the drone behaves (heights, speeds, limits, waypoints).**
→ [`data/flightsettings.json`](data/flightsettings.json). Every tunable lives there. Put it on the drone with `pio run -e fly -t uploadfs`.

**I want to add a new flight mode (phase).**
1. Add it to the *end* of the `FlightPhase` enum in [`FlightModel.hpp`](src/flight/models/FlightModel.hpp) (+ its name in `phaseName`).
2. Add per-phase state structs in [`PhaseState.hpp`](src/flight/state/PhaseState.hpp) and to `SharedState`.
3. Create `phases/YourPhase.hpp` implementing [`IFlightPhase`](src/flight/phases/IFlightPhase.hpp) (copy an existing phase as a template).
4. Register it in [`PhaseRegistry.hpp`](src/flight/phases/PhaseRegistry.hpp) — include it, and add one line to `buildPhases()`: `table[PHASE_YOURS] = new YourPhase(motors, ...);` (and size the table by your phase, since it's now the last one).
5. Add a `transitionTo(PHASE_YOURS, ...)` somewhere that should trigger it.
([`ManualPhase`](src/flight/phases/ManualPhase.hpp) is a recent, complete example of exactly these steps.)

**I want to tune the flight feel (wobbly, sluggish, drifty).**
→ The PID gains in the `airframe` section of [`flightsettings.json`](data/flightsettings.json) (loaded by [`MotorController::configure`](src/flight/services/MotorController.hpp)). Raise a P gain for a snappier response, add D to reduce overshoot. Test the change in the on-chip sim (`run_hil.sh`) before flying.

**I want to change what a radio switch does.**
→ [`RcInput.hpp`](src/flight/services/RcInput.hpp) (the `crsfHandle...` functions) and the channel map (`radio.channels`) in [`flightsettings.json`](data/flightsettings.json).

**I want a new test run or tool (like a bench check or a calibration).**
→ Make it a new app: a folder in [`src/apps/`](src/apps/) with a `main.cpp`, plus an `[env:your_app]` in [`platformio.ini`](platformio.ini). Use the `fc::` API and/or the parts from `realHardware()`; never add a test mode inside the flight controller.

**I want a new sim test (a new failure to try).**
→ A new file in [`src/apps/sim/scenarios/`](src/apps/sim/scenarios/): change the world, wrap one plug, or be the pilot (see [section 8](#8-simulation--testing)).

**I want to support a different sensor chip (say, another GPS).**
→ Write one driver in [`src/flight/hardware/real/`](src/flight/hardware/real/) that implements that plug (`IGps`), and change one line in [`realHardware()`](src/flight/hardware/HardwareIo.hpp) (`io.gps = new YourGps();`). Nothing else changes.

**I want to understand a specific log message.**
→ Search the string; every log line is a `logLine("...")`. Remember these strings double as the sim test protocol, so don't reword them casually.

---

## 10. Glossary

- **ESC** — Electronic Speed Controller. The board that drives one motor. You send 0.0–1.0; it spins the motor.
- **PWM (ESC signal)** — the classic pulse-width signal every ESC understands (1000–2000 µs). Implemented in [`EspPwmESC.hpp`](src/flight/hardware/real/EspPwmESC.hpp).
- **IMU** — Inertial Measurement Unit (accelerometer + gyroscope). Tells you tilt and rotation.
- **Madgwick filter** — the math that fuses accel + gyro into stable angles.
- **PID** — the target-vs-actual control formula. See [`PID.hpp`](src/flight/services/PID.hpp).
- **Mutex** — the "pen"; a lock that lets only one core touch shared data at a time.
- **FreeRTOS / task** — the mini-OS that runs the parallel loops ("tasks") on the two cores.
- **Phase** — one self-contained flight behavior (PARKED, MISSION, …).
- **Failsafe** — an automatic safety reaction (return home / land) when something's wrong.
- **RTL** — Return To Launch. The come-home autopilot.
- **CRSF / ELRS** — the radio protocol / radio system your transmitter uses.
- **Geofence** — an invisible max-distance circle around the launch point.
- **SITL** — Software-In-The-Loop: the physics simulation runs on the same chip as the firmware (here, QuadSim inside the flight loop). See the [`sim` app](src/apps/sim/main.cpp).
- **QuadSim** — the standalone C++ quad-physics library in [`lib/QuadSim/`](lib/QuadSim/) that powers the on-chip sim.
- **Waypoint** — a GPS point (lat/lon/altitude) the mission flies to.
- **App** — a small flashable program (`src/apps/*`) that uses the flight controller: `fly`, `first_mission`, `bench_test`, `esc_calibration`, `compass_calibration`, `sim`.
- **FlightIo / plug** — one small interface per sensor, plus motors and radio, that the flight controller reads and drives through: the real drivers from `realHardware()` on the real drone, fakes in the simulator.
- **Scenario** — one sim test: it changes the world, breaks one plug, and/or plays the pilot.
- **flightsettings.json** — every tunable setting, loaded from the drone's flash at power-up (like `appsettings.json` in C#).

---

## 11. Honest notes & rough edges

Because this code was largely AI-written, here are a few things a newcomer should know so they aren't confused:

- **`data/` also holds old web files.** `index.html`, `script.js` and `style.css` are leftovers from an early web-server version. Nothing uses them; they just ride along when the settings are uploaded.
- **IMU field names are a little wrong.** In [`Mpu6050Imu.hpp`](src/flight/hardware/real/Mpu6050Imu.hpp), the fused roll/pitch/yaw angles are stored in fields named `gyroX/gyroY/gyroZ`. They're angles, not raw gyro rates. This naming flows through the whole codebase. The one true rate is `yawRateDps` (deg/s, + = clockwise), which the yaw damping uses. Its sign assumes the MPU6050 is mounted flat and right side up; check it on the bench (turn the drone clockwise by hand → it should read positive).
- **The altitude controller uses a hover feed-forward.** A baseline throttle (`airframe.hoverThrottle` in [`flightsettings.json`](data/flightsettings.json)) holds the drone up and the altitude PID only trims around it — this replaced the old integral-windup-from-zero approach that made the height hunt up and down. Tune it per airframe.
- **Phases intentionally duplicate code.** The near-identical `physicsTick()` in each phase is a deliberate choice (full isolation, so one phase can't break another), not an oversight. Resist the urge to "DRY" them into a base class unless you really mean to change that design decision.
- **The altitude is ground-referenced twice.** [`EspBarometer`](src/flight/hardware/real/EspBarometer.hpp) sets a ground-pressure reference, then [`Bme280Altimeter`](src/flight/hardware/real/Bme280Altimeter.hpp) averages ~2 s more readings and subtracts that too. Harmless (the second offset is ~0), just redundant.
- **The sim turns the nose for the drone.** QuadSim's yaw isn't flown by the flight controller's yaw loop; [`SimWorld`](src/flight/hardware/sim/SimWorld.hpp) points the nose at the current target itself (up to 90°/s). So the sim doesn't test yaw control, only everything that depends on the heading.

Welcome aboard — start with the [`fly` app](src/apps/fly/main.cpp) and [`FlightController.hpp`](src/flight/FlightController.hpp), then follow the [guided tour](#5-a-guided-tour-one-whole-flight). 🚁
