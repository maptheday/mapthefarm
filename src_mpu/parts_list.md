# Drone Parts List — F450 Build, ESP32 Brain

*Based on Hoarder Sam's [$150 autonomous drone](https://www.youtube.com/watch?v=uC9hVyqGvDE) ([his parts sheet](https://docs.google.com/spreadsheets/d/1cw73QHCPfB9ar2uMawwrjXqWUJa2Jb6L4nhbwQM9KqE/edit?pli=1&gid=0#gid=0)), with one big change: **your ESP32 and your firmware are the flight controller**, instead of his SpeedyBee board running INAV. Updated 2026-09-26.*

*The AliExpress links (and prices) are in his sheet. AliExpress requires a login, so I couldn't check those listings myself. Shipping is usually 1–3 weeks.*

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

Everything else (frame, motors, ESCs, props, GPS, receiver, battery) is **exactly his list**.

---

## Drone parts

Change ☐ to ✅ as things arrive.

| Bought | Part | Where | Price | Notes |
|:---:|---|---|---|---|
| ☐ | F450 frame kit | his sheet (AliExpress) | ~$20 | Same size as the drone in your sim. |
| ☐ | 2212 1400KV motors ×4 | his sheet | ~$20 ($5 each) | Get 2 CW + 2 CCW if the listing offers it. |
| ☐ | 40A ESCs ×4 | his sheet | ~$20 ($5 each) | **Probably PWM-only, not DShot** (see "Code changes"). If the listing says the ESC has a "BEC" or "5V output", note it; it may power the ESP32 and save the UBEC. |
| ☐ | 8045 props | his sheet | ~$5 | Buy a spare set. |
| ☐ | M100-5883 GPS + compass | his sheet | ~$20 | GPS **and** compass in one. See the GPS check below. |
| ☐ | ELRS 2.4 GHz receiver | his sheet | ~$10 | Outputs CRSF, which is what `RcInput.hpp` reads. |
| ☐ | 3S ~5000 mAh LiPo, XT60 | his sheet | ~$30 | Get the XT60 plug version. |
| ☐ | 5V UBEC | [Readytosky 5V/3A, Amazon](https://www.amazon.com/Readytosky-Module-Adjustable-Switchable-Quadcopter/dp/B083HVVBR2) or AliExpress | ~$6 | Battery → 5V for the ESP32, GPS, and receiver. Set it to **5V** before connecting. Skip it if the ESCs have a 5V BEC. |
| ✅ | ESP32-S3 dev board | already owned | — | The brain: runs your firmware. |
| ✅ | MPU6050 | already owned | — | Tilt sensor. Exactly what `Imu.hpp` uses. |
| ✅ | BME280 | already owned | — | Height sensor. Exactly what the barometer code uses. |

**Drone parts still to buy: about $131**, cheaper than his $150, because your ESP32 replaces the $40 SpeedyBee.

## Support gear (skip what you already own)

| Bought | Part | Where | Price | Notes |
|:---:|---|---|---|---|
| ☐ | Charger | [ISDT PD60, USB-C, XT60 (Pulse Battery)](https://www.pulsebattery.com/products/isdt-pd60-60w-6a-battery-balance-charger-type-c-input-for-1-4s-lipo-battery) | $19.99 | In stock as of 2026-09-26. Needs a **USB-C PD adapter, 60W+**; most USB-C laptop chargers work. Otherwise add ~$15–20. |
| ☐ | Radio | RadioMaster Pocket ELRS (his sheet), or [BETAFPV LiteRadio 3 ELRS](https://betafpv.com/products/literadio-3-radio-transmitter) $32.99 | $33–90 | Any ELRS 2.4 GHz radio works with the receiver. The Pocket is nicer; the LiteRadio is cheaper but ships slowly (12–30 days). |
| ☐ | Soldering iron | Pinecil (his sheet) | ~$40 | Skip if you have a temperature-controlled iron. |
| ☐ | M3 standoffs + screws | his sheet | ~$10 | For mounting the ESP32 and sensors. |
| ☐ | LiPo bag | Amazon / AliExpress | ~$10 | See the safety section. |
| ☐ | Cell checker with buzzer | Amazon / AliExpress | ~$6 | Your fuel gauge: the firmware doesn't watch battery voltage. |
| ☐ | Smoke stopper (XT60) | Amazon / AliExpress | ~$8 | For the first power-up after soldering. |
| ☐ | Small ABC fire extinguisher | hardware store | ~$20 | For the field (see "At the field"). |

---

## Code changes this build needs

I can make all of these when you're ready:

1. **PWM motor driver.** Your `EspESC.hpp` speaks DShot600, but cheap 40A ESCs almost always only understand plain **PWM** (the classic servo-style signal). The ESP32 has a hardware PWM unit (LEDC) that makes this simpler than DShot. Every ESC understands PWM, so this also works if the ESCs turn out to be DShot-capable after all.
2. **One-time ESC calibration.** PWM ESCs need to learn the throttle range once: full-throttle signal at power-up, then minimum. Always **props off**. I'll add a small calibration step.
3. **Quick check that your BME280 is genuine.** Cheap "BME280" boards are sometimes relabeled BMP280s. If yours is one, the code stops at boot with a barometer error. It's easy to test by reading the chip's ID over I2C (0x60 = BME280, 0x58 = BMP280). If it turns out to be a BMP280, it still works fine; it just needs a small library change.
4. **GPS check when it arrives.** M100-class GPS modules often run at **38400** baud (your code says 9600: `GPS_BAUD` in `FlightConfig.hpp`). The compass is usually a **QMC5883L**, which your code expects, but some newer modules use a **QMC5883P**, a different chip that would need a different library. Check the listing or the chip marking.
5. **Update the sim thrust** once you weigh the real drone. The size already matches.

## Charging and safety (don't skip)

### 🧸 Why LiPo batteries need respect

A LiPo packs a lot of energy into a soft foil pouch. Treated right, it's as safe as the battery in your phone. Treated wrong (overcharged, punctured, crushed, or drained too far), it can swell up and catch fire, and a LiPo fire is hot, fast, and hard to put out, because the chemicals feed it once it starts.

The good news: almost every LiPo fire comes from breaking one of a handful of simple rules. The gear below exists to make those rules easy to follow.

### What's inside a "3S" battery

"3S" means **three cells in a row** ("S" = series). Each cell is its own little battery, and each one has a safe range:

```text
  cell 1      cell 2      cell 3
 [ 3.7V ] + [ 3.7V ] + [ 3.7V ]  = 11.1V "nominal"

 4.20 V per cell   FULL. Never go above this (overcharged = fire risk).
 3.80 V per cell   STORAGE. Where a battery should sit when you're not flying.
 3.50 V per cell   LAND NOW. Stop flying around here.
 3.00 V per cell   EMPTY. Below this the cell is damaged and may swell.
```

The small white plug with 4 wires on the battery is the **balance lead**. It connects to each cell individually, so a charger can see (and fix) every cell, not just the total.

### The gear, and what each piece protects you from

| Part | Pick | What it does, in plain words |
|---|---|---|
| Balance charger | the ISDT PD60 (in the table above) | Charges each cell to exactly 4.20 V. A cheap charger that only watches the total (12.6 V) can let one cell hit 4.3 V while another sits at 4.1 V. That overcharged cell is how fires start. |
| LiPo safe bag | any fireproof LiPo bag, sized for a 5000 mAh pack | Charge and store batteries inside it. If a battery fails, the bag holds in the flames and sparks long enough for you to deal with it. It slows a fire down; it doesn't make one harmless. |
| Cell checker / low-voltage alarm | any "LiPo cell checker with buzzer" (a few dollars) | Plugs into the balance lead and shows each cell's voltage. Before a flight: "is it full?" After: "did I drain it too far?" **Your firmware doesn't watch battery voltage at all.** The only guard is the 5-minute flight limit (`MAX_FLIGHT_TIME_MS`), so this is your fuel gauge. Some can ride on the drone and beep when a cell gets low. |
| Smoke stopper | XT60 smoke stopper | Goes between battery and drone for the **first power-up after any soldering**. If a wire is shorted, it lets through only a trickle of current, so a mistake glows a little bulb instead of burning your ESCs. Remove it before flying. |
| Fire plan | a metal bucket of dry sand, near where you charge | If a battery starts to puff, hiss, or smoke, don't grab it with bare hands. If you safely can, use tongs to move the bag onto concrete outside, or smother it with sand. If the fire spreads, get out and call 911. |

### Charging, step by step

1. **Look first.** Is the battery puffy, dented, or torn? If so, **don't charge it**. Retire it (see below).
2. **Let it cool.** After a flight, wait about 15 minutes. Never charge a warm battery.
3. **In the bag, on a hard surface.** Put the bag on concrete, tile, or a baking sheet, away from curtains, paper, and the couch.
4. **Plug in BOTH leads:** the big XT60 (power) *and* the white balance lead.
5. **Set the charger:** type **LiPo**, **3S**, mode **Balance charge**, current **4.5 A** (see "1C" below; 4.5 A is the most the PD60 can push into a 3S pack). Double-check the screen shows 3 cells before pressing start.
6. **Stay nearby.** Never charge while you're asleep or out of the house.
7. **When it's done,** unplug both leads.

**What "1C" means:** charge current equal to the battery's capacity. 5000 mAh = 5.0 A. The PD60 tops out around 4.5 A on 3S, so a full charge takes a bit over an hour. It's the safe default. Faster charging is possible, but it heats the battery and wears it out.

### Storing and retiring batteries

- **Not flying for 2+ days?** Run the charger's **storage** mode. It takes each cell to 3.8 V. LiPos left full for days slowly puff up and lose capacity.
- **Always unplug the battery from the drone** after flying. The ESP32 and UBEC keep drawing a little power and will drain it below 3.0 V overnight.
- **Never** leave batteries in a hot car, and never puncture or crush one.
- **Retiring a battery** (puffy, damaged, or won't hold a charge): run the charger's discharge mode to drain it, tape over the connector, and take it to a battery-recycling drop-off. Many hardware stores have one. **Never** put it in the trash.

### At the field

The real fire risk outdoors isn't the drone burning. It's **dry grass or crops catching** from it.

**Don't wrap the battery in anything fireproof on the drone.** A LiPo heats up while flying and needs air to cool; wrapping it traps heat and makes a failure *more* likely. A box strong enough to hold in a LiPo fire would be too heavy to fly. Instead, protect it from crashes, the main cause of in-flight LiPo fires:

- Mount it **inside** the frame (between plates or on top), not hanging underneath where it hits the ground first.
- Strap it down, with a grippy pad, so it can't be thrown in a crash.
- Use landing legs tall enough to keep the pack off the ground.
- Optional upgrade: a **hard-case** 3S 5000 mAh pack resists crushing (for example, [Rlaarlo, $59.99](https://rlaarlo.com/products/5000mah-lipo-battery-hardcase)).
- Land before cells drop below about 3.5 V. The 5-minute flight limit leaves plenty of margin on a 5000 mAh pack.

**Bring and do:**

- Fly over short green grass or bare dirt, not dry stubble, hay, or ripe crops. Skip burn-ban days and very dry, windy days.
- Carry a small **ABC fire extinguisher**, or a water jug and a shovel. The battery burns out in a minute or two on its own; your job is to stop the **grass** around it from spreading.
- Carry batteries to and from the field in the LiPo bag.
- **After any crash:** if the pack is smoking, stay back, let it burn out, and put out the grass around it. If it's calm, unplug it right away, set it on bare dirt away from anything that burns, and **watch it for 15 minutes**. Damaged packs can catch fire minutes later. Don't reuse a dented or puffy pack.
- The firmware helps: the geofence keeps the drone over the field, and GPS loss makes it land rather than wander.

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

Rough weight: frame ~280 g + motors ~210 g + ESCs ~100 g + battery ~380 g + props ~40 g + electronics and wiring ~80 g ≈ **1,090 g**. The sim assumes 1,200 g ✓ close.

Thrust: about 850 g per motor (2212 1400KV, 8045 props, 3S) × 4 ≈ 3.4 kg, so thrust-to-weight is **about 3:1**, versus the sim's 4:1. Expected hover throttle ≈ √(1 ÷ 3) ≈ **0.58**, versus the current `HOVER_THROTTLE_FF` of 0.50. Weigh the finished drone, update the sim, then find the true hover throttle on a tether.

Flight time on 5000 mAh should be long: the video claims 20+ minutes. Your firmware's 5-minute limit (`MAX_FLIGHT_TIME_MS`) is very safe for first flights; raise it later, once you trust the cell checker readings.
