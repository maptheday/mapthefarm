# How All The Hardware Pieces Work Together

A plain-English guide to what each part does, how it connects, and why it matters.

---

## The Big Picture

Think of the drone like a person riding a bicycle:

- The **ESP32-S3** is the brain and spinal cord
- The **MPU6050** is the inner ear (tells you if you're leaning)
- The **ESC** is the legs (makes things spin)
- The **Motors** are the wheels
- The **nRF24L01** is the ears (listens for your commands)
- The **Battery** is the stomach (feeds everything energy)
- The **BMP280** is the eyes looking down (how high am I?)

---

## Each Part Explained

---

### ESP32-S3 — The Brain

**What it does:**
This is the microcontroller chip that runs your C++ flight controller code. It has two CPU cores running at 240 MHz. Your `runCore0_Avionics()` runs on Core 0, your `runCore1_Flight()` runs on Core 1 — simultaneously, all the time.

**What plugs into it:**
Everything else plugs into this. It's the center of the spider web.

**The pins that matter:**
```
GPIO 4  → Motor 1 signal wire
GPIO 5  → Motor 2 signal wire
GPIO 6  → Motor 3 signal wire
GPIO 7  → Motor 4 signal wire
GPIO 8  → SDA (I2C data line) → MPU6050 + BMP280
GPIO 9  → SCL (I2C clock line) → MPU6050 + BMP280
GPIO 14 → nRF24L01 CE pin
GPIO 11 → nRF24L01 MOSI
GPIO 12 → nRF24L01 MISO
GPIO 13 → nRF24L01 SCK
3.3V    → Powers MPU6050, BMP280, nRF24L01
GND     → Shared ground for everything
```

**In your code:**
This is where `main.cpp` runs. The `setup()` function boots everything up. Then `runDualCore()` splits into two threads that never stop.

---

### MPU6050 — The Inner Ear

**What it does:**
A tiny chip that measures which way it's tilting (accelerometer) and how fast it's spinning (gyroscope). It answers the question: *is the drone level right now?*

Without this, the flight controller is completely blind. It has no idea the drone is tipping over.

**What plugs into it:**
```
VCC  → 3.3V from ESP32
GND  → GND
SDA  → GPIO 8 on ESP32  (data)
SCL  → GPIO 9 on ESP32  (clock)
AD0  → GND              (sets its address to 0x68)
```

SDA and SCL are the I2C bus — a two-wire system where the ESP32 sends a message saying "hey 0x68, give me your latest readings" and the MPU6050 sends back the numbers.

**In your code:**
`EspIMU.hpp` talks to this chip. It reads 14 bytes over I2C, converts them to degrees, and feeds `roll_angle`, `pitch_angle`, and `yaw_angle` into your flight loop every 4ms.

**The catch with yaw:**
The MPU6050 has no magnetometer (compass). It knows roll and pitch perfectly from the accelerometer. But yaw (which direction the drone is pointing) can only be estimated by integrating the gyroscope over time — which drifts. If you hover for 5 minutes the yaw reading may be off by 10-20 degrees. A magnetometer (like the QMC5883L that Cortex uses) fixes this.

---

### 4-in-1 ESC — The Motor Controller

**What it does:**
ESC stands for Electronic Speed Controller. It takes a digital command from the ESP32 and converts it into the three-phase AC power that brushless motors need to spin. You can't connect a brushless motor directly to a battery and make it spin — you need an ESC in between.

A "4-in-1 ESC" is just four ESCs on one board, one for each motor. Tidier wiring.

**What plugs into it:**
```
Battery leads  → Your LiPo battery (thick red and black wires)
Motor wires    → 3 wires per motor (order determines spin direction)
Signal wire 1  → GPIO 4 on ESP32
Signal wire 2  → GPIO 5 on ESP32
Signal wire 3  → GPIO 6 on ESP32
Signal wire 4  → GPIO 7 on ESP32
GND            → Shared GND with ESP32 (important — must be connected)
```

**In your code:**
`EspESC.hpp` talks to this using DShot600 — a digital protocol that sends 16-bit frames over the ESP32's RMT peripheral. `MotorMixer.hpp` calculates what percentage to send to each motor. `EspESC` converts that 0.0-1.0 number into a DShot frame and fires it out GPIO 4/5/6/7.

**DShot vs normal PWM:**
Your original `EspESC.hpp` used standard PWM (like an old servo). The new DShot version is digital — cleaner signal, no calibration needed, doesn't drift. Most modern ESCs support DShot600. Check your ESC spec sheet before buying.

**Motor spin direction:**
The three wires going from ESC to motor can be connected in any order, but the order determines which way the motor spins. If a motor spins the wrong way, swap any two of the three wires. No code change needed.

---

### Brushless Motors — The Wheels

**What they do:**
Convert electrical power into spinning. Each motor spins a propeller. The speed difference between motors is what makes the drone move.

```
         FRONT
    M1(CCW)  M2(CW)
       \      /
        \    /
        /    \
       /      \
    M3(CW)  M4(CCW)
         REAR
```

CCW = counter-clockwise. CW = clockwise. Opposite pairs cancel each other's torque so the drone doesn't spin on its own.

**What plugs into them:**
Three wires per motor → into the ESC. No direct connection to the ESP32.

**In your code:**
Motors don't appear in your code directly. Your code talks to the ESC, and the ESC talks to the motors.

---

### nRF24L01 — The Ears

**What it does:**
A 2.4 GHz radio module. It receives commands from your transmitter (a handheld controller, or the Synapse transmitter that Cortex uses). Without this, you have no way to tell the drone what to do.

**What plugs into it:**
```
VCC   → 3.3V from ESP32  (note: this chip is sensitive to power — add a 10µF cap here)
GND   → GND
CE    → GPIO 14 on ESP32
CSN   → GPIO 9  on ESP32
MOSI  → GPIO 11 on ESP32
MISO  → GPIO 12 on ESP32
SCK   → GPIO 13 on ESP32
```

These are SPI pins — a four-wire high-speed communication bus. Different from I2C (two wires). SPI is faster and used for real-time data like radio commands.

**In your code:**
This isn't implemented yet in your flight controller. It's the next major piece to add. When implemented, the radio task runs on Core 0 and writes commands into the IPC bridge so Core 1 can act on them.

---

### BMP280 — The Altimeter

**What it does:**
Measures air pressure. Since air pressure drops predictably as you go higher, you can calculate altitude from pressure. Good to about ±1 metre accuracy outdoors.

**What plugs into it:**
```
VCC  → 3.3V from ESP32
GND  → GND
SDA  → GPIO 8 on ESP32  (same I2C bus as MPU6050)
SCL  → GPIO 9 on ESP32
SDO  → GND              (sets its address to 0x76)
```

It shares the I2C bus with the MPU6050. The ESP32 talks to each by using different addresses (0x68 for MPU6050, 0x76 for BMP280).

**In your code:**
`EspBarometer.hpp` reads this. Your altitude PID in Core 0 uses it to hold a target height. As noted earlier this is optional for basic flight — you can fly manually without it.

**The limitation:**
Barometers are affected by prop wash (air turbulence from your own propellers). Cortex actually uses a LiDAR for altitude hold because it's more accurate close to the ground. The barometer is better for higher altitudes.

---

### LiPo Battery — The Stomach

**What it does:**
Powers everything. A 3S LiPo means 3 lithium cells in series = 11.1V nominal (12.6V fully charged, 9V minimum before damage).

**What plugs into it:**
```
Thick leads  → ESC power input (the ESC distributes power to motors)
Balance lead → LiPo charger (for charging, not used during flight)
```

The ESP32 gets power from the ESC's built-in BEC (Battery Eliminator Circuit) which steps down the 11.1V battery voltage to 5V for the electronics.

**In your code:**
Battery voltage isn't monitored yet. Cortex uses an INA219 current sensor for this. Flying without voltage monitoring means you won't get a low-battery warning — you'll just notice the drone getting sluggish.

---

## How A Single Flight Cycle Works

Here's what happens in the 50ms between each Core 0 tick:

```
1. BMP280 sends altitude reading over I2C (GPIO 8/9)
   → EspBarometer reads it
   → altitude_pid calculates throttle correction

2. MPU6050 sends roll/pitch/yaw over I2C (GPIO 8/9)
   → EspIMU reads it
   → IPC bridge updated with target commands

3. Core 1 (running at 250Hz between Core 0 ticks):
   → Reads IPC bridge
   → MPU6050 read again for latest attitude
   → roll_pid, pitch_pid, yaw_pid calculate corrections
   → MotorMixer combines throttle + roll + pitch + yaw
   → EspESC fires DShot frames out GPIO 4/5/6/7
   → ESC converts to 3-phase power
   → Motors spin at commanded speed
   → Propellers push air
   → Drone moves
```

---

## Wiring Summary

```
┌─────────────────────────────────────────────────┐
│                  ESP32-S3                        │
│                                                  │
│  GPIO 4  ──────────────────── ESC Motor 1 signal│
│  GPIO 5  ──────────────────── ESC Motor 2 signal│
│  GPIO 6  ──────────────────── ESC Motor 3 signal│
│  GPIO 7  ──────────────────── ESC Motor 4 signal│
│                                                  │
│  GPIO 8 (SDA) ─────┬────────── MPU6050 SDA      │
│                    └────────── BMP280 SDA        │
│  GPIO 9 (SCL) ─────┬────────── MPU6050 SCL      │
│                    └────────── BMP280 SCL        │
│                                                  │
│  GPIO 14 ──────────────────── nRF24L01 CE        │
│  GPIO 11 ──────────────────── nRF24L01 MOSI      │
│  GPIO 12 ──────────────────── nRF24L01 MISO      │
│  GPIO 13 ──────────────────── nRF24L01 SCK       │
│                                                  │
│  3.3V ─────┬──────────────── MPU6050 VCC         │
│            ├──────────────── BMP280 VCC          │
│            └──────────────── nRF24L01 VCC        │
│                                                  │
│  GND  ─────┬──────────────── MPU6050 GND         │
│            ├──────────────── BMP280 GND          │
│            ├──────────────── nRF24L01 GND        │
│            └──────────────── ESC GND (signal)    │
└─────────────────────────────────────────────────┘

Battery → ESC (thick power wires)
ESC → Motors (3 wires per motor)
ESC → ESP32 5V + GND (powers the electronics)
```

---

## What You Still Need To Implement

| Thing | Status | Notes |
|---|---|---|
| EspESC (DShot) | ✅ Written | CRC calculation needs fixing |
| EspIMU | ✅ Written | readSensorData() called twice — minor fix needed |
| EspBarometer | ✅ Written | Looks correct |
| nRF24L01 radio driver | ❌ Missing | Needed to receive flight commands |
| Radio task (Core 0) | ❌ Missing | FreeRTOS task to poll radio |
| Battery monitoring | ❌ Missing | Optional but important for safety |

The radio is the most important missing piece. Without it the drone can only run pre-programmed missions (like your current MissionCommander sequence), not respond to a human pilot.