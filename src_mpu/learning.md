### Does the MPU have a gyroscope and an accelerometer?

**Yes, absolutely.** The MPU6050 is what the electronics world calls a **6-DoF (Six Degrees of Freedom) IMU (Inertial Measurement Unit)**.

Inside that tiny black chip, there are actually two completely separate sensor systems baked into the same piece of silicon:

1. **A 3-axis Accelerometer:** Measures acceleration (forces acting on it, including gravity) along the X, Y, and Z axes.
2. **A 3-axis Gyroscopes:** Measures angular velocity (rotational speed) around the X, Y, and Z axes.

That is why your original code contains distinct events for both `gyro_readings` and `accelerometer_readings`. They are two different tools trying to tell you different things about how your device is moving.

---

### Stepping Back: Ramping Up the Math (Slowly)

Let's demystify why dividing by `50`, `70`, or `90` is causing your 3D cube to lose its mind, and look at how we actually track position.

Imagine you are driving a car, and your dashboard speedometer is broken, but you have a stopwatch. You want to know exactly how far down the road you have traveled.

If you travel at a perfectly steady speed of **60 mph** for exactly **1 hour**, the math is incredibly easy:


$$\text{Distance} = 60 \text{ mph} \times 1 \text{ hour} = 60 \text{ miles}$$

In physics, multiplying your *speed* by *time* to find your *position* is called **integration**.

Now, let's look at your gyroscope. The gyroscope doesn't tell you your angle; it only tells you your **rotational speed** (e.g., *"I am spinning at 10 degrees per second right now"*). To find your total angle, you have to multiply that speed by how long the sensor has been spinning.

#### The Flaw in Your Current Code

Your code does this:

```cpp
gyroX += gyroX_temp / 50.00;

```

Dividing by `50.00` is the mathematical equivalent of multiplying by `0.02`. By doing this, your code is aggressively assuming: *"The time that has passed since my last reading is exactly 0.02 seconds (20 milliseconds)."*

But Microcontrollers (like the ESP32) are constantly multitasking. Between your sensor readings, the ESP32 is handling Wi-Fi handshakes, processing web server requests, and sending data to your browser.

* On loop #1, it might take **20ms** to read the sensor. (Dividing by 50 is correct!)
* On loop #2, a Wi-Fi event happens, and it takes **80ms** to read the sensor.
* Because your code still divides by `50.00` (assuming 20ms passed), it completely misses the extra 60ms of movement.

#### The Fix: Tracking Real Time ($dt$)

To stop your project from getting out of sync, you have to measure *exactly* how much time passed since the last loop. In programming, we call this change in time **Delta-Time** ($dt$).

Instead of hardcoding `50.00`, your code needs to act like a real stopwatch:

```cpp
unsigned long currentTime = millis(); // What time is it now?
float dt = (currentTime - previousTime) / 1000.0; // How many seconds actually passed?
previousTime = currentTime; // Save the time for the next loop

// Now multiply your speed by the actual time passed:
gyroX += gyroX_temp * dt; 

```

By multiplying by the true $dt$, your math scales perfectly, whether the ESP32 took 5 milliseconds or 100 milliseconds to loop back around.

---

### How is your drone going to stay stable?

If your math is currently drifting, the thought of launching a drone with spinning blades sounds terrifying! How does a drone handle this without flying into a wall?

Drones stay stable using a continuous software feedback loop called a **PID Controller** (Proportional-Integral-Derivative).

Think of a PID controller like driving a car and trying to stay centered in your lane:

* **The Input (Where you want to be):** You want the drone to be perfectly level (0 degrees).
* **The Sensor (Where you actually are):** The MPU6050 calculates the drone's current angle (e.g., tilted 5 degrees to the left).
* **The Error:** The difference between the two (5 degrees off track).
* **The Correction:** The PID controller instantly calculates how much faster the left motors need to spin compared to the right motors to force the drone back to 0 degrees.

This calculation happens **400 to 1,000 times every single second**. The drone is constantly making micro-adjustments so fast that to the human eye, it looks like it is just floating perfectly still.

To build a stable drone, you *cannot* use the raw, drifting gyro math you currently have. You will need to use a **Sensor Fusion library** (like the *Madgwick* or *Mahony* filter libraries available in Arduino) which combines the accelerometer and gyroscope to give the PID loop a rock-solid, drift-free angle reading.

---

### Do you need GPS on your initial flight? (With MPU & BME)

**No, you do not need GPS for your initial flight.** In fact, it is highly recommended that you do *not* use GPS at first.

Here is what your current sensor suite does:

* **MPU6050 (Gyro/Accel):** Keeps the drone level. It stops it from flipping upside down and crashing instantly.
* **BME (Barometric Pressure Sensor):** Measures changes in air pressure to determine **altitude**. It tells the drone how high it is above the ground so it can hold its height.

With just the MPU and BME, your drone will be capable of **Altitude Hold**. It will hover at a set height, and it will keep itself flat.

#### The Catch: The "Drift" in the Air

Because you don’t have a GPS or a downward-facing camera, the drone has no idea where it is *geographically*. If a gust of wind blows against the drone, it will tilt to stay level, but the wind will physically push it sideways. The drone will slowly drift across the yard with the wind, and you will have to manually use your remote control joysticks to fight the wind and keep it in place.

**Your Initial Flight Plan:**

1. Find a wide-open, grassy field on a day with **zero wind**.
2. Start in manual/altitude-hold mode using just your MPU and BME.
3. Master keeping the drone in a stable hover yourself.
4. Once your stabilization math is proven to work, *then* you can add a GPS module to introduce "Position Hold" (where the drone fights the wind automatically to lock itself to a specific coordinate).