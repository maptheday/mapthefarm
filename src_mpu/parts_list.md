# Drone Parts List — F450 Build, ESP32 Brain

*Based on Hoarder Sam's [$150 autonomous drone](https://www.youtube.com/watch?v=uC9hVyqGvDE) ([his parts sheet](https://docs.google.com/spreadsheets/d/1cw73QHCPfB9ar2uMawwrjXqWUJa2Jb6L4nhbwQM9KqE/edit?pli=1&gid=0#gid=0)), with one big change: **your ESP32 and your firmware are the flight controller**, instead of his SpeedyBee board running INAV. Updated 2026-09-26.*

*The AliExpress links below are the ones from his sheet and video description (his affiliate links). AliExpress requires a login, so I couldn't check those listings myself; prices are his. Shipping is usually 1–3 weeks.*

---

## What changes from his list

| | His build | Your build |
|---|---|---|
| Brain | SpeedyBee F405 Wing Mini + INAV ($40) | **your ESP32-S3 + your firmware** ✅ already owned |
| Tilt sensor | built into the SpeedyBee | MPU6050 ✅ already owned |
| Height sensor | built into the SpeedyBee | BME280 ✅ already owned |
| 5V power for electronics | built into the SpeedyBee | small 5V UBEC (~$6) |
| Payload drop + servo | included | optional, skip for now |
| Charger | Hota D6 ($90) | **ISDT PD60 ($19.99)** |
| Battery | 3S 5000 mAh LiPo (~$30) | **2× 3S 2100 mAh LiFePO4 ($80)**: much less likely to catch fire, lighter, shorter flights |

Everything else (frame, motors, ESCs, props, GPS, receiver, battery) is **exactly his list**.

---

## Drone parts

Change ☐ to ✅ as things arrive.

| Bought | Part | Where | Price | Notes |
|:---:|---|---|---|---|
| ☐ | F450 frame kit | [AliExpress](https://s.click.aliexpress.com/e/_c4CX4GQL) | ~$20 | Same size as the drone in your sim. |
| ☐ | 2212 1400KV motors ×4 | [AliExpress](https://s.click.aliexpress.com/e/_c3qMQMPz) | ~$20 ($5 each) | Get 2 CW + 2 CCW if the listing offers it. |
| ☐ | 40A ESCs ×4 | [Readytosky 40A 2-4S with 5V/3A BEC (AliExpress)](https://www.aliexpress.us/item/2251832810662122.html). Buy **4**. | check listing | The original link in his sheet is dead, and the Amazon 4-pack is out of stock (as of 2026-09-26). This is the common F450 ESC. Maker's specs: 2-4S, **5V/3A BEC**, about 33 g each, and it "is not able to flash the firmware", i.e. plain **PWM**, which is exactly what `EspPwmESC.hpp` sends. The BEC can power the ESP32, so the UBEC below becomes optional (see the wiring rule under it). |
| ☐ | 8045 props | [AliExpress](https://s.click.aliexpress.com/e/_c3dzmgQx) | ~$5 | Buy a spare set. |
| ☐ | M100-5883 GPS + compass | [AliExpress](https://s.click.aliexpress.com/e/_c2vAzZ8L) | ~$20 | GPS **and** compass in one. See the GPS check below. |
| ☐ | ELRS 2.4 GHz receiver | [AliExpress](https://s.click.aliexpress.com/e/_c2Q2zSAx) | ~$10 | Outputs CRSF, which is what `RcInput.hpp` reads. |
| ☐ | **2×** 3S 2100 mAh **LiFePO4** batteries | [Pulse 2100mAh 3S 9.9V 25C LiFePO4, XT60 (Boomerang RC Jets, Indiana)](https://www.boomerangrcjets.com/products/pulse-2100mah-3s-9-9v-25c-receiver-lifepo4-battery-xt60-connector) | $39.99 each | **The safer battery chemistry** (see the safety section). 168 g, XT60 + balance lead, 25C (≈52 A, plenty). About 6–8 minutes of flying per pack, so two packs = two flights per charge session. Cheaper alternative when in stock: [Liperior 3S 2100 LiFePO4](https://rcbattery.com/liperior-2100mah-3s-25c-9-9v-lifepo4-receiver-battery-pack-with-xt60-plug.html), $23.99. *(Replaces his 3S 5000 mAh LiPo.)* |
| ☐ | 5V UBEC (optional) | [Readytosky 5V/3A, Amazon](https://www.amazon.com/Readytosky-Module-Adjustable-Switchable-Quadcopter/dp/B083HVVBR2) or AliExpress | ~$6 | **Probably not needed:** the Readytosky ESCs have a built-in 5V BEC. **Wiring rule if you use the ESC's BEC:** each ESC's 3-wire signal plug has a red 5V wire. Connect the red wire from **only one** ESC to the ESP32's 5V pin, and pull the red wire out of the other three plugs (tape it back). Four BECs tied together can fight each other. The ground (black/brown) and signal (white/yellow) wires from all four still connect. Buy the UBEC only if you'd rather keep the electronics' power separate. |
| ✅ | ESP32-S3 dev board | already owned | — | The brain: runs your firmware. |
| ✅ | MPU6050 | already owned | — | Tilt sensor. Exactly what `Imu.hpp` uses. |
| ✅ | BME280 | already owned | — | Height sensor. Exactly what the barometer code uses. |

**Drone parts still to buy: about $180–200**, depending on the ESC price. That's more than his $150, because two LiFePO4 packs ($80) replace his single $30 LiPo. The extra ~$50 buys a much lower fire risk (you can also skip the UBEC).

## Support gear (skip what you already own)

| Bought | Part | Where | Price | Notes |
|:---:|---|---|---|---|
| ☐ | Charger | [ISDT PD60, USB-C, XT60 (Pulse Battery)](https://www.pulsebattery.com/products/isdt-pd60-60w-6a-battery-balance-charger-type-c-input-for-1-4s-lipo-battery) | $19.99 | In stock as of 2026-09-26. Needs a **USB-C PD adapter, 60W+**; most USB-C laptop chargers work. Otherwise add ~$15–20. |
| ☐ | Radio | [RadioMaster Pocket ELRS (AliExpress)](https://s.click.aliexpress.com/e/_c4BrEQfd), or [BETAFPV LiteRadio 3 ELRS](https://betafpv.com/products/literadio-3-radio-transmitter) $32.99 | $33–90 | Any ELRS 2.4 GHz radio works with the receiver. The Pocket is nicer; the LiteRadio is cheaper but ships slowly (12–30 days). |
| ☐ | Soldering iron | [Pinecil (AliExpress)](https://s.click.aliexpress.com/e/_okrM6IL) | ~$40 | Skip if you have a temperature-controlled iron. |
| ☐ | M3 standoffs + screws | [standoffs](https://www.aliexpress.us/item/3256803016697405.html), [thin-head screws](https://www.aliexpress.us/item/3256806779956979.html) (AliExpress) | ~$10 | For mounting the ESP32 and sensors. |
| ☐ | Battery bag (sold as a "LiPo bag"; works for LiFe) | Amazon / AliExpress | ~$10 | See the safety section. |
| ☐ | Cell checker with buzzer | Amazon / AliExpress | ~$6 | Your fuel gauge: the firmware doesn't watch battery voltage. |
| ☐ | Smoke stopper (XT60) | Amazon / AliExpress | ~$8 | For the first power-up after soldering. |
| ☐ | Small ABC fire extinguisher | hardware store | ~$20 | For the field (see "At the field"). |

---

## Code changes this build needs

I can make all of these when you're ready:

1. ✅ **PWM motor driver (done).** `EspPwmESC.hpp` sends standard PWM (1000–2000 µs, 400 times a second), which every ESC understands, so it works whether or not the ESCs support DShot. If the motors stutter or won't arm, try `ESC_PWM_HZ = 50` in `FlightConfig.hpp` (some plane ESCs only accept 50).
2. ✅ **One-time ESC calibration (done, you run it once).** With **props off**: set `CALIBRATE_ESCS_ON_BOOT = true` in `FlightConfig.hpp`, flash, open the serial monitor (115200), and follow the prompts (type `GO`, plug in the battery, type `MIN` after the beep-beep). It halts when finished. Then set it back to `false` and flash again.
3. ✅ **Bench-test mode (done).** `BENCH_TEST_ON_BOOT` in `FlightConfig.hpp`: spin one motor at a time and stream every sensor, props off. Used in Stage 6 of the build guide.
4. ✅ **Motor spin directions fixed (done).** The old diagram in `IESC.hpp` had the directions backward for how the mixer steers yaw, which would have made yaw control push the wrong way. Build with **M1 + M4 clockwise, M2 + M3 counter-clockwise** (the guide's Stage 2).
5. **Quick check that your BME280 is genuine.** Cheap "BME280" boards are sometimes relabeled BMP280s. If yours is one, the code stops at boot with a barometer error. It's easy to test by reading the chip's ID over I2C (0x60 = BME280, 0x58 = BMP280). If it turns out to be a BMP280, it still works fine; it just needs a small library change.
6. **GPS check when it arrives.** M100-class GPS modules often run at **38400** baud (your code says 9600: `GPS_BAUD` in `FlightConfig.hpp`). The compass is usually a **QMC5883L**, which your code expects, but some newer modules use a **QMC5883P**, a different chip that would need a different library. Check the listing or the chip marking.
7. **Update the sim thrust** once you weigh the real drone. The size already matches.

## 🛠️ Build guide: from boxes to first flight

*For a first-time builder. Go in order, one stage per sitting. Each stage ends with a ✅ check; don't move on until it passes.*

### The five safety rules (read these first)

1. **Props OFF** for everything until the very last stage. A spinning prop cuts skin like a blade.
2. **Battery UNPLUGGED** whenever you solder, screw, or move wires.
3. **Smoke stopper** between battery and drone for the first power-up after any soldering.
4. **USB power first, battery second.** Plug the ESP32 into your laptop, check the messages, *then* plug in the battery.
5. **Red is +, black is −.** Check twice before plugging anything in. Backwards battery wiring destroys ESCs instantly.

### The big picture

```text
                        ┌──────────────── BATTERY (3S, 11.1V) ───────────────┐
                        │                                                     │
                 bottom plate of the frame = power board (spreads battery power)
                  │            │            │            │
                ESC 1        ESC 2        ESC 3        ESC 4      (each turns battery power
                  │            │            │            │         into spin for one motor)
               MOTOR 1      MOTOR 2      MOTOR 3      MOTOR 4
                  ▲            ▲            ▲            ▲
                  └── signal wires (how fast?) from the ESP32 ──┘

   ESP32 (the brain) ◄── MPU6050 (tilt)     ◄── BME280 (height)
                     ◄── GPS + compass      ◄── radio receiver (your sticks and switches)
```

Two kinds of wiring: **thick power wires** (battery → ESCs → motors, lots of current) and **thin signal wires** (between the ESP32 and everything else, tiny current).

---

### Stage 0 — Set up your bench

☐ A clear table with good light, and a heat-safe mat for soldering.
☐ Tools: soldering iron, solder, flux, heat-shrink, wire cutters/strippers, small screwdrivers (hex keys for the frame), zip ties, multimeter (optional but very handy, ~$15).
☐ Watch the soldering and frame sections of [Hoarder Sam's video](https://www.youtube.com/watch?v=uC9hVyqGvDE). Same frame, motors, and ESCs, so his hands-on steps apply. Only the "flight controller" part is different: yours is the ESP32.

**🧸 Soldering in 30 seconds:** solder is metal glue that melts at ~370 °C. Heat the *parts* with the iron tip (not the solder), then touch the solder to the parts; it flows in. A good joint is shiny and cone-shaped, like a tiny volcano. A dull, lumpy ball means it didn't get hot enough: reheat it. Slide heat-shrink onto the wire *before* soldering, then shrink it over the joint.

---

### Stage 1 — Power: the bottom plate and ESCs

The F450's bottom plate is a circuit board. Its copper pads spread battery power to all four arms.

1. **Battery lead:** solder the XT60 pigtail to the big center pads: **red → +**, **black → −**.
2. **ESC power:** each ESC has a thick red and black wire on its "battery" side. Solder each ESC to the pads near one arm: **red → +**, **black → −**.
3. Leave the ESCs' other ends (three motor wires, one thin signal plug) loose for now.

✅ **Check (battery unplugged):** if you have a multimeter, set it to "continuity" (beep mode). Touching the XT60's + and − pins must **not** beep. A beep means a short circuit: find it before going on.

---

### Stage 2 — Frame, motors, ESC mounting

1. **Mark the front.** F450 kits usually have two red arms and two white arms. Call the **red arms the front**. Write "FRONT" on a piece of tape on the top plate.
2. **Screw the arms** between the bottom plate and top plate.
3. **Mount the motors** on the arm ends, using the short screws from the motor box. ⚠️ Screws that are too long touch the motor's copper windings and destroy the motor. The screw should only go ~3 mm into the motor.
4. **Which motor goes where.** Motors come labeled CW or CCW (this sets the prop nut's thread so it tightens itself as the motor spins):

```text
                FRONT (red arms)
        M1  CW  ⟳            ⟲  CCW  M2
             ╲                ╱
              ╲    [ESP32]   ╱
              ╱              ╲
             ╱                ╲
        M3  CCW ⟲            ⟳  CW   M4
                    REAR
   (spin direction seen from ABOVE)
```

5. **Connect each motor to its ESC:** three bullet plugs, in any order for now. Stage 6 checks the direction, and fixing it is just swapping two wires.
6. **Zip-tie each ESC** flat under its arm, away from the prop's path.

✅ **Check:** every motor spins freely by hand, with no scraping sounds.

---

### Stage 3 — Mount the electronics

1. **ESP32:** on nylon standoffs on the top plate, center of the frame, with the USB port reachable.
2. **MPU6050 (tilt sensor):** as close to the **center** of the drone as possible, **flat**, on a square of double-sided foam tape. The foam soaks up motor vibration, which otherwise looks like shaking to the sensor. Write down which way its printed arrows point. Stage 6 checks whether the orientation is right.
3. **BME280 (height sensor):** anywhere on the top plate, with a small piece of open-cell foam (like a sponge) taped loosely over it. Wind from the props and direct sunlight both confuse air-pressure readings.
4. **GPS + compass:** on a short mast (the kit or his 3D files have one) on top, **arrow pointing to the FRONT**, as far as you can get it from the battery and power wires. Electric current makes its own magnetic field and confuses the compass.
5. **Receiver:** anywhere, with its antenna(s) sticking out, not covered by carbon or metal.

---

### Stage 4 — Wiring the brain

Here's where every wire goes. **"TX goes to RX":** a device *talks* on its TX pin and *listens* on its RX pin, so one device's TX connects to the other's RX.

| From | Wire | ESP32 pin | Why |
|---|---|---|---|
| ESC 1 (front-left) signal | white/yellow | **GPIO 4** | motor 1 speed |
| ESC 2 (front-right) signal | white/yellow | **GPIO 5** | motor 2 speed |
| ESC 3 (rear-left) signal | white/yellow | **GPIO 6** | motor 3 speed |
| ESC 4 (rear-right) signal | white/yellow | **GPIO 7** | motor 4 speed |
| All 4 ESCs | black/brown (ground) | **GND** | shared reference |
| **One** ESC only | red (5V from its BEC) | **5V** | powers the ESP32. Pull the red wire out of the other 3 plugs and tape it back. |
| MPU6050 | SDA / SCL | **GPIO 16** / **GPIO 15** | I2C bus (shared) |
| BME280 | SDA / SCL | **GPIO 16** / **GPIO 15** | same bus |
| GPS compass | SDA / SCL | **GPIO 16** / **GPIO 15** | same bus |
| MPU6050, BME280 | VCC / GND | **3V3** / **GND** | |
| GPS | TX / RX | **GPIO 17** / **GPIO 18** | GPS talks on TX → ESP32 listens on 17 |
| GPS | VCC / GND | **5V** / **GND** | check the listing says 5V is OK (most say 3.3–5V) |
| Receiver | TX | **GPIO 8** | your sticks and switches |
| Receiver | 5V / GND | **5V** / **GND** | |

**🧸 The I2C bus** is like a party line: three sensors share the same two wires (SDA carries the data, SCL keeps the beat), and each answers to its own address. That's why SDA and SCL appear three times. A small piece of perf board with a row for SDA, a row for SCL, a row for 3V3, and a row for GND makes this tidy.

✅ **Check:** trace each wire against the table with your finger, one row at a time.

---

### Stage 5 — First power-up (props OFF)

1. **USB only first.** Plug the ESP32 into your laptop and open the serial monitor (115200). You should see the startup messages, including `[ESC] PWM channels initialized`. If the drone halts with an IMU, compass, or barometer error, recheck that sensor's four wires. (A barometer error could also mean your "BME280" is really a BMP280.)
2. **Battery, through the smoke stopper.** Plug in the battery with the smoke stopper in between. If its bulb glows bright and stays bright, **unplug at once** and look for a short. A quick flash is normal.
3. The ESCs play a startup tune. That's good: it means they're getting power *and* a signal.

---

### Stage 6 — Software setup on the bench (props OFF)

Each step below is a flag in [`FlightConfig.hpp`](src/state/FlightConfig.hpp): set it to `true`, flash, follow the serial monitor, then set it back to `false`.

1. **☐ ESC calibration** (`CALIBRATE_ESCS_ON_BOOT`): teaches the ESCs the throttle range. Type `GO`, plug in the battery, type `MIN` after the beep-beep. Done once.
2. **☐ Motor order and direction** (`BENCH_TEST_ON_BOOT`, then type `1`, `2`, `3`, `4`): each motor spins slowly for 2 seconds.
   - Wrong **corner** spinning? That ESC's signal wire is on the wrong GPIO pin.
   - Wrong **direction**? Unplug the battery and swap any two of that motor's three wires.
   - Trick for seeing direction: touch a strip of paper *lightly* against the motor's side. It flicks in the spin direction.
3. **☐ Sensor directions** (same bench test, type `s`): tilt the drone by hand and watch:
   - Nose **up** → pitch goes **positive**.
   - Right side **down** → roll goes **positive**.
   - Turn it **clockwise** → heading goes **up** and yaw rate is **positive**.
   - Lift it ~1 m → height goes up ~3 ft.

   If roll or pitch goes the wrong way, the MPU6050 is rotated relative to what the code expects. Tell me which way each number moves, and we'll fix it in code or by turning the sensor.
4. **☐ Compass calibration** (`CALIBRATE_COMPASS_ON_BOOT`): do it **outdoors**, away from cars and metal. Slowly rotate the drone through every orientation (every side facing down, then spin it around) for 30 seconds. Saved permanently.
5. **☐ GPS:** take it outside, run the bench test `s`, and wait (the first fix can take a few minutes). You want "sats" of 8 or more and a real latitude/longitude. If you see nothing at all, the baud rate may be 38400: change `GPS_BAUD` in `FlightConfig.hpp`.
6. **☐ Radio:** bind the receiver to your radio (the receiver's manual; usually plug in power 3 times quickly, then "Bind" on the radio). Set up the switches on these channels:

| Radio channel | Job | Rule |
|---|---|---|
| 5 | **START** | flip UP: take off, then (flip again) start the mission |
| 6 | **STOP** | flip DOWN: **motors cut instantly**, any time. Keep it UP to fly. |
| 7 | **MANUAL** | UP: you fly with the sticks |

   ✅ With props off and the battery in, flip STOP down: the log should show the stop, and nothing can spin until it's back up.

---

### Stage 7 — Props on, first flights

1. **Props:** each prop has a direction. The thicker, rounded edge must **lead**, cutting into the air in the spin direction, and the curved (scooped) side faces **up**. Put CW props on M1/M4 and CCW props on M2/M3. Tighten the nuts firmly.
2. **First hover: tethered.** Tie the drone to something heavy with a short rope (a cinder block works), in an open area. Finger on the **STOP** switch the whole time.
3. **Watch for:**
   - Spins in circles → yaw direction problem. **STOP**, then tell me.
   - Flips over immediately on takeoff → a motor in the wrong corner or spinning backward. **STOP**, then redo Stage 6.
   - Sinks or shoots up → hover throttle is off. Adjust `HOVER_THROTTLE_FF` (about 0.59 expected).
4. **Only after several calm tethered hovers:** untethered low hovers, then short missions in the field. Follow the "At the field" rules below.

> **If the radio link drops** (out of range, radio battery dies): after 1 second of silence the drone **comes home and lands by itself** (RTL). If it's lower than 5 ft or has no GPS, it **lands where it is** instead. Test it once on the tether, hovering **below 5 ft** (so it lands instead of climbing 60 ft against the rope): switch the radio off, and within about a second it should start landing. Switch the radio back on right away so STOP works again.

---

## Charging and safety (don't skip)

> **Want the full picture?** [lipo_battery_course.md](lipo_battery_course.md) is a whole beginner course on drone batteries: why they catch fire, the habits that prevent it, and why this build uses LiFePO4. The section below is the quick version.

### 🧸 Why LiFePO4 instead of LiPo

Most drones use **LiPo** batteries: the most energy for their weight, but if they're overcharged, over-drained, or crushed in a crash, they can burst into flames. This drone uses **LiFePO4** ("LiFe"), a steadier chemistry. Abused or damaged, a LiFe pack is **far less likely to catch fire**. It's more likely to just swell, get hot, or vent smoke. The trade is weight: about half the energy for the same weight, so shorter flights.

That's the right trade for a drone flying over someone's crops. **"Less likely" still isn't "never",** though: the habits below still apply.

### What's inside a "3S" LiFePO4 battery

"3S" means **three cells in a row**. LiFe cells use **lower voltages than LiPo**. Never charge a LiFe pack on a LiPo setting (that would overcharge it).

```text
  cell 1      cell 2      cell 3
 [ 3.3V ] + [ 3.3V ] + [ 3.3V ]  = 9.9V "nominal"

 3.65 V per cell   FULL. The charger stops here. (Pack: ~11.0 V)
 3.30 V per cell   STORAGE. Where a battery should sit when you're not flying.
 3.00 V per cell   LAND NOW (at rest).
 2.50 V per cell   EMPTY. Below this the cell is damaged.
```

**LiFe's quirk: its voltage barely moves until it's nearly empty.** Most of the flight it sits around 3.2–3.3 V per cell, then it drops off a cliff at the end. So the voltage **can't tell you "half full"**. Your fuel gauge is the **flight timer**, and the charger's "mAh put back in" number. The firmware's 5-minute limit (`MAX_FLIGHT_TIME_MS`) fits comfortably inside one pack.

The small white plug on the battery is the **balance lead**. It lets the charger see (and even out) each cell separately.

### The gear, and what each piece protects you from

| Part | Pick | What it does, in plain words |
|---|---|---|
| Balance charger | the ISDT PD60 (in the table above) | Charges each cell to exactly 3.65 V, watching every cell through the balance lead so no single cell gets overcharged. |
| Battery bag | any fireproof LiPo bag (they work for LiFe too) | Charge, store, and carry batteries in it. It holds in heat, sparks, and smoke if a pack fails. It slows a failure down; it doesn't make one harmless. |
| Cell checker | any "LiPo cell checker" (reads LiFe cells too) | Plugs into the balance lead and shows each cell's voltage. Check the pack is full (~3.4–3.6 V/cell resting) before flying, and that no cell has drifted away from the others. |
| Smoke stopper | XT60 smoke stopper | Goes between battery and drone for the **first power-up after any soldering**. If a wire is shorted, it lets through only a trickle of current, so a mistake glows a little bulb instead of burning your ESCs. Remove it before flying. |
| Fire plan | a bucket of dry sand near where you charge, and an ABC extinguisher for the field | If a pack puffs up, hisses, or smokes, don't grab it bare-handed. Move the bag outside with tongs if it's safe to, or smother it. If anything else catches, get out and call 911. |

### Charging, step by step

1. **Look first.** Is the battery puffy, dented, or torn? If so, **don't charge it**. Retire it (see below).
2. **Let it cool.** After a flight, wait about 15 minutes. Never charge a warm battery.
3. **In the bag, on a hard surface.** On tile or concrete, away from curtains, paper, and the couch.
4. **Plug in BOTH leads:** the XT60 (power) *and* the white balance lead.
5. **Set the charger:** type **LiFe** (⚠️ not LiPo), **3S**, mode **Balance charge**, current **2.1 A**. Check the screen shows 3 cells before pressing start.
6. **Stay nearby.** Never charge while you're asleep or out of the house.
7. **When it's done,** unplug both leads.

**Why 2.1 A:** "1C" means a charge current equal to the battery's capacity: 2100 mAh = 2.1 A, about an hour per charge. That's the gentle, safe default.

### Storing and retiring batteries

- **Not flying for a week or more?** Run the charger's **storage** mode (set to **LiFe**): about 3.3 V per cell. LiFe tolerates sitting full much better than LiPo, but storage mode is still kinder.
- **Always unplug the battery from the drone** after flying. The ESP32 and ESC keep drawing a little power and will drain it too low overnight.
- **Never** leave batteries in a hot car, and never puncture or crush one.
- **Retiring a battery** (puffy, damaged, or won't hold a charge): drain it with the charger's discharge mode, tape over the plugs, and take it to a battery-recycling drop-off. **Never** put it in the trash.

### At the field

LiFePO4 makes a field fire much less likely, but the habits still matter. The real outdoor risk is **dry grass or crops catching**.

- **Mount the battery inside the frame** (between the plates or on top), strapped down with a grippy pad, not hanging underneath where it hits the ground first.
- **Fly over short green grass or bare dirt,** not dry stubble, hay, or ripe crops. Skip burn-ban days and very dry, windy days.
- **Carry a small ABC fire extinguisher** in the truck, or a water jug and a shovel.
- **Carry batteries in the bag** to and from the field.
- **After any crash:** if the pack is smoking, stay back and deal with the grass around it. If it's calm, unplug it right away, set it on bare dirt away from anything that burns, and **watch it for 15 minutes**. Don't reuse a dented or puffy pack.
- **The firmware helps:** the geofence keeps the drone over the field, losing GPS makes it land, and losing the radio makes it come home or land.

### First power-up of the finished drone

1. **Props OFF.** Always, for any bench work.
2. Smoke stopper between battery and drone.
3. Plug in. If the smoke stopper's bulb glows brightly and stays bright, unplug immediately and hunt for the short.
4. If all is calm, remove the smoke stopper and continue testing, still with props off.

## Build supplies

- Soldering iron (temperature-controlled), solder, flux, heat-shrink
- XT60 connectors, 14 AWG silicone wire (power), 22-26 AWG wire (signals)
- M3 nylon standoffs and screws (to mount the ESP32 above the power board)
- Dupont jumper wires, a small perf board (ties the I2C sensors together)
- Battery strap, zip ties, double-sided foam tape
- Tether line and a heavy anchor for first hover tests

---

## How the real build compares to the sim

Rough weight: frame ~280 g + motors ~210 g + ESCs ~130 g + battery ~170 g + props ~40 g + electronics and wiring ~80 g ≈ **910 g**, lighter than the sim's 1,200 g.

Thrust: about 850 g per motor on a 3S LiPo (11.1 V). A 3S LiFePO4 is 9.9 V, and thrust grows roughly with voltage squared, so expect about 850 × (9.9 ÷ 11.1)² ≈ 680 g per motor, ≈ 2.7 kg total. Thrust-to-weight is then **about 2.9:1** (less voltage, but a lighter drone), versus the sim's 4:1. Expected hover throttle ≈ √(1 ÷ 2.9) ≈ **0.59**, versus the current `HOVER_THROTTLE_FF` of 0.50. Weigh the finished drone, update the sim, then find the true hover throttle on a tether.

Flight time: a 2100 mAh LiFe pack holds about 21 Wh. Using at most 80% of it at roughly 130 W of hover power gives **about 6–8 minutes** per pack (an estimate; time your real flights). Your firmware's 5-minute limit (`MAX_FLIGHT_TIME_MS`) fits inside that with margin, so leave it at 5 minutes for this battery.
