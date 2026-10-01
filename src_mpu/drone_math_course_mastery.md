# The Drone Math Course

*Every piece of math inside your drone's firmware, explained from scratch. Each idea starts as an everyday picture and ends as a real line of code in this repo.*

---

## Before you start

**Who this is for:** someone who took Calc 1–3 a long time ago, remembers that the words exist, and has lost the connections between them. That's normal. You don't need to relearn how to think. You just need the pictures back.

**How every lesson works.** Each lesson follows the same five beats, so you always know where you are:

| Beat | What it does |
|---|---|
| 🧸 **ELI5** | an everyday picture with no math in it |
| 🖼️ **Picture** | the same idea as a diagram |
| 🔢 **Numbers** | one small example, worked all the way through |
| 💻 **In the code** | the exact line in this repo that does it |
| ✍️ **Your turn** | one problem, with the answer right below it (cover it up first) |

Each lesson ends with a **one-line takeaway**. If you can say the takeaway out loud and explain it, you're ready for the next lesson. If you can't, re-read the ELI5, not the formula.

**The route through the course:**

```text
 Part 0   The big loop ............ what all this math is FOR
 Part 1   Trig .................... splitting arrows, and un-splitting them (sin, cos, atan2)
 Part 2   Vectors ................. arrows as lists of numbers (add, length, dot product, projection)
 Part 3   Calculus refresher ...... slope and area, the two ideas behind PID
 Part 4   PID ..................... "how hard should I push?"
 Part 5   Motor mixing ............ turning four wishes into four motor speeds
 Part 6   GPS ..................... turning latitude/longitude into meters
 Part 7   Line following .......... the "carrot" that keeps the drone on the line
 Part 8   Forces .................. why it hovers, why it falls (F = ma)
 Part 9   Rotation ................ torque, spin, and why it tips
 Part 10  Orientation ............. frames, rotating arrows, quaternions
 Part 11  One whole tick .......... every lesson, running together
 Part 12  Practice, reference sheet, study routine
```

Parts build on each other. Part 1 is the foundation for almost everything, so it's the longest. Don't rush it.

---

# Part 0 — The big loop

## Lesson 0.1 — What is all this math *for*?

### 🧸 ELI5

Balance a broomstick upright on your palm. You never hold still. You're constantly:

1. **looking** at the broom ("it's leaning left"),
2. **deciding** how to fix it ("move my hand left, a little"),
3. **moving** your hand,
4. and immediately **looking again**, because the broom has moved.

You do this dozens of times a second without thinking. A drone does exactly the same thing, 200 times a second, and every one of those four steps is a piece of math.

### 🖼️ Picture

```text
        ┌──────────────────────────────────────────────────────────┐
        │                                                          │
        ▼                                                          │
   ① SENSE           ② DECIDE              ③ ACT                  │
   "Where am I?"      "What should I do?"   "Spin the motors"      │
   GPS, compass,  ─►  navigation (Part 6-7) ─► motor mixing  ─►  ④ PHYSICS
   IMU, barometer     PID (Part 4)             (Part 5)          "What actually
   (Part 10)          trig (Part 1)                               happens" (Part 8-9)
                      vectors (Part 2)                                 │
                                                                       │
                                            new position & tilt ───────┘
```

Every lesson in this course lives in one of those boxes. When you get lost, come back to this picture and ask "which box am I in?"

### 💻 In the code

This loop really exists. In [`FlightController.hpp`](src/flight/FlightController.hpp) there are two tasks:

```text
navigationTask   every 100 ms (10×/sec)   slow thinking: "which way to the waypoint?"
physicsTask      every 5 ms  (200×/sec)   fast thinking: "keep me level, set the motors"
```

That 5 ms number is called **`dt`** ("delta time", the time step). You'll see `dt` in almost every formula from Part 3 onward.

**Takeaway:** *The drone runs sense → decide → act → physics in a loop, forever. All the math serves one of those four steps.*

---

## Lesson 0.2 — Units: your free bug detector

### 🧸 ELI5

If someone tells you "the drone is 5 away," your first question is "5 *what*?" Five meters? Five degrees? Five seconds? A number with no unit is a sentence with no noun.

### 🔢 Numbers

Units follow the same algebra as numbers. They multiply and cancel:

```text
20 m ÷ 4 s          = 5 m/s          (distance ÷ time = speed)
5 m/s × 2 s         = 10 m           (speed × time = distance; the "s" cancels)
3 m/s² × 2 s        = 6 m/s          (acceleration × time = speed)
```

The rule that catches bugs: **you can only add or subtract things with the same unit.** `3 m + 2 m` is fine. `3 m + 2 m/s` is nonsense, like adding apples to Tuesdays.

### 💻 In the code

Suppose you saw this in a physics update:

```text
position += acceleration * dt
```

Write the units underneath:

```text
   m     +=    m/s²     ×  s
   m     +=    m/s                ✗  can't add m and m/s
```

It's broken, and you found it without running anything. The correct chain (Part 8) is two steps:

```text
velocity += acceleration * dt       m/s += m/s² × s = m/s   ✓
position += velocity     * dt       m   += m/s  × s = m     ✓
```

That's exactly what QuadSim does in [`QuadSim.h`](lib/QuadSim/src/QuadSim.h):

```cpp
s_.vel[0] += ax * h;          // h is QuadSim's name for dt
s_.pos[0] += s_.vel[0] * h;
```

Watch out for units in *names* in this codebase, too: `targetAltFt` is feet, `distM` is meters, `rollRad` is radians, `roll` is degrees. The suffix is a promise about the unit.

**Takeaway:** *Write the units under every equation. If the two sides don't match, the equation is wrong.*

---

# Part 1 — Trigonometry: splitting arrows

## Lesson 1.1 — Sine and cosine are an arrow-splitter

### 🧸 ELI5

Stand in a field at noon, with the sun directly overhead, holding a stick at an angle. Look at its **shadow on the ground**. That shadow is how much of the stick points *sideways*.

Now imagine a flashlight shining at it from the side, throwing a shadow onto a wall. That shadow is how much of the stick points *up*.

One slanted stick, two shadows. **Cosine and sine are just the lengths of those two shadows.** That's all they are. Nothing mystical.

### 🖼️ Picture

```text
                        ● tip of the stick
                       /│
                      / │
         stick,      /  │  ← the "wall shadow":
         length 1   /   │     how much points UP
                   /    │
                  / θ   │
                 ●──────┘
                 ← the "ground shadow":
                    how much points ALONG the ground
```

If the stick has length **1**, then:

```text
ground shadow  =  cos(θ)
wall shadow    =  sin(θ)
```

And if the stick has some other length, say `L`, both shadows just scale up:

```text
ground shadow  =  L · cos(θ)
wall shadow    =  L · sin(θ)
```

That second pair is the most-used formula in this whole course. Every time the drone has "an arrow of some length, pointing at some angle," it uses those two lines to split it into two pieces it can work with.

### 🔢 Numbers

Here are the angles worth knowing by feel. Don't memorize the digits. Look at the *pattern*:

```text
angle     cos (ground shadow)    sin (wall shadow)     the stick is...
  0°         1.000                  0.000               lying flat
 30°         0.866                  0.500               tipped up a bit
 45°         0.707                  0.707               exactly halfway
 60°         0.500                  0.866               mostly up
 90°         0.000                  1.000               standing straight up
```

As the stick rises, the ground shadow shrinks and the wall shadow grows. At 45° they're equal, because the stick is exactly halfway between.

A stick of length 10 at 30°:

```text
ground shadow = 10 × cos(30°) = 10 × 0.866 = 8.66
wall shadow   = 10 × sin(30°) = 10 × 0.500 = 5.00
```

**Sanity check (always do this):** the two shadows are the legs of a right triangle, and the stick is the long side. Pythagoras says they must fit back together:

```text
√(8.66² + 5.00²) = √(75 + 25) = √100 = 10   ✓
```

### ✍️ Your turn

A stick of length 4 at 60°. How long is each shadow? Does it pass the Pythagoras check?

> **Answer:** ground = 4 × 0.5 = **2**, wall = 4 × 0.866 = **3.46**. Check: √(2² + 3.46²) = √(4 + 12) = √16 = 4 ✓. At 60° the stick is mostly up, so the wall shadow is the bigger one. ✓

**Takeaway:** *`L·cos(θ)` and `L·sin(θ)` split one slanted arrow into two straight pieces.*

---

## Lesson 1.2 — The one rule: cosine is the piece *along* your starting line

This is the lesson that prevents 90% of trig confusion. Read it twice.

### 🧸 ELI5

In Lesson 1.1 we measured the angle **up from the ground**, so cosine was the ground shadow. But what if you measured the stick's angle **from straight up** instead ("it's leaning 30° away from vertical")? Then cosine becomes the *up* shadow.

Cosine didn't change meaning. You changed where you started measuring. The rule is:

> **Cosine = the piece that lies ALONG the line you measured the angle from.**
> **Sine = the piece that sticks OUT sideways from that line.**

Don't memorize "cosine is horizontal." That's only true for the classroom picture.

### 🖼️ Picture

```text
  Angle measured FROM THE GROUND            Angle measured FROM STRAIGHT UP
  (classroom habit)                         (how a drone's tilt works)

              ●                                   up
             /│                                   ▲
            / │ sin = sticks out                  │ θ  ●
           /  │   (up)                            │   /
          / θ │                           cos =   │  /
         ●────┘                           along   │ /    sin = sticks out
         cos = along                      (up)    │/       (sideways)
         (ground)                                 ●──────►
```

### 🔢 Numbers

This course uses three starting lines. Same rule every time:

```text
angle measured FROM...           cosine lands on...    where you'll see it
the ground (classroom)           the ground piece      this lesson
straight up (drone tilt)         the UP piece          Lesson 1.4
north (compass bearing)          the NORTH piece       Lessons 1.8 and 6.3
```

### ✍️ Your turn

A drone's compass bearing is measured **from north**, turning toward east. For an arrow 100 m long at bearing 30°, which piece is `100·cos(30°)`, the north piece or the east piece?

> **Answer:** the **north** piece, because we measured from north. So north = 100 × 0.866 = 86.6 m, east = 100 × 0.5 = 50 m. Sanity check: 30° is close to north, so most of the arrow should point north. 86.6 > 50 ✓

**Takeaway:** *Always ask "which line did I measure from?" That piece gets cosine. The other gets sine.*

---

## Lesson 1.3 — Degrees vs. radians (and that weird `0.0174532925f`)

### 🧸 ELI5

Degrees and radians are two units for the same thing, like inches and centimeters. People like degrees (a circle is 360). Math libraries insist on radians. So code converts back and forth constantly.

What *is* a radian? Take a circle. Take a piece of string exactly as long as the circle's radius. Lay that string along the edge of the circle. The angle it covers is **1 radian**, a bit under 60°.

### 🖼️ Picture

```text
                     ● ╮
                    ╱   ╲  ← a string as long as the radius,
                   ╱     │   bent along the circle's edge
                  ╱ 1rad │
                 ●───────●
                 center   radius

   the angle at the center that the string covers = 1 radian ≈ 57.3°
```

How many strings fit all the way around? The circumference is `2π × radius`, so **2π strings**, about 6.28. That gives you the only conversion you need:

```text
full circle  =  360°  =  2π radians
half circle  =  180°  =   π radians
1°           =  π/180 radians  =  0.0174532925 radians
```

### 🔢 Numbers

```text
degrees → radians:   multiply by π/180  (≈ 0.01745)
radians → degrees:   multiply by 180/π  (≈ 57.2958)

 30° = 0.524 rad      45° = 0.785 rad      90° = 1.571 rad (π/2)      180° = 3.142 rad (π)
```

### 💻 In the code

In [`MotorController.hpp`](src/flight/services/MotorController.hpp):

```cpp
float rollRad  = roll  * 0.0174532925f;   // degrees × (π/180) = radians
float pitchRad = pitch * 0.0174532925f;
float tiltFactor = 1.0f / (cosf(rollRad) * cosf(pitchRad));
```

That magic number is just π/180. `cosf()` expects radians, so if you passed it `30` meaning 30°, it would compute the cosine of 30 *radians* (about 1,719°) and give garbage. Elsewhere the code uses Arduino's helpers, `radians(x)` and `degrees(x)`, which do the same multiplication. QuadSim uses `57.29577951f`, which is 180/π, going the other way.

**Takeaway:** *C math functions speak radians. Multiply degrees by π/180 (0.01745) before calling `sin`/`cos`.*

---

## Lesson 1.4 — Why tilting a drone makes it sink

### 🧸 ELI5

Push a lawnmower. If you push straight down on the handle, all your effort goes into the ground and nothing moves forward. Push straight forward and it all goes into rolling. Hold the handle at an angle and your push **splits**: some goes down, some goes forward.

A drone's propellers always push along the drone's own "up" direction. When the drone tilts, that push tilts with it and splits into **up** (holding it in the air) and **sideways** (moving it). That's actually *how a quadcopter flies somewhere*: it tilts toward where it wants to go. The catch is that the "up" piece shrinks.

### 🖼️ Picture

The tilt angle `θ` is measured **from straight up** (level = 0°). By the rule from Lesson 1.2, cosine is the up piece:

```text
     LEVEL (θ = 0°)                 TILTED (θ = 30°)

          ▲ T                          up
          │                            ▲ θ   ↗ T (same length)
          │                   T·cos θ  │   ╱
          │                  (holds    │  ╱
      [ drone ]               it up)   │ ╱
                                       │╱
   all of T holds it up               ●────► T·sin θ  (pushes it sideways)
```

### 🔢 Numbers

Thrust `T = 10 N` (N is Newtons, a unit of force; about 1 kg of weight is 9.8 N).

```text
tilt    up piece = 10·cos(θ)     sideways piece = 10·sin(θ)
  0°       10.00 N                   0.00 N
 10°        9.85 N                   1.74 N
 30°        8.66 N                   5.00 N
 45°        7.07 N                   7.07 N
 60°        5.00 N                   8.66 N
```

Notice the first few degrees are almost free: at 10° you lose only 1.5% of your lift but gain real sideways push. That's why drones cruise at small tilts. At 60°, half of your lift is gone.

### ✍️ Your turn

A drone makes 20 N of thrust and needs 18 N of "up" to hover. It tilts 30°. Does it hold height, or sink?

> **Answer:** up piece = 20 × cos(30°) = 20 × 0.866 = **17.3 N**. That's less than the 18 N it needs, so it **sinks**. To fix that, it would need to throttle up. That's the next lesson.

**Takeaway:** *A tilted drone keeps only `cos(tilt)` of its thrust pointing up.*

---

## Lesson 1.5 — Tilt compensation: reading the real code

This is the first time the math becomes a real line of firmware. Here's the code from [`MotorController.hpp`](src/flight/services/MotorController.hpp):

```cpp
float rollRad  = roll  * 0.0174532925f;
float pitchRad = pitch * 0.0174532925f;
float tiltFactor = 1.0f / (cosf(rollRad) * cosf(pitchRad));
tiltFactor = constrain(tiltFactor, 1.0f, 2.0f);   // 2.0 ~= 60 deg of tilt
out.baseThrottle = constrain(out.baseThrottle * tiltFactor, 0.0f, 1.0f);
```

We'll build every piece.

### 🧸 ELI5

You're carrying a heavy box up a ramp, and someone tilts the ramp. You automatically lean in and push harder to keep the box moving up. This code is the drone "pushing harder" by exactly the right amount when it tilts, so it doesn't sag.

### Piece 1 — where `1 / cos` comes from

We'll do this with numbers first. The letters come at the very end.

**The situation.** The drone weighs 10 N, so it needs **10 N pointing up** to hover.

- **Level:** it pushes 10 N, and all of it points up. Hovers ✓.
- **Tilted 30°:** only `cos(30°) = 0.866` of its push points up (Lesson 1.4). If it keeps pushing 10 N, only `10 × 0.866 = 8.66 N` points up. That's short of 10, so it **sinks**.

**The question:** how hard does it need to push, so that after it loses the tilt's share, **10 N still points up**?

#### 🧸 ELI5: the paycheck

This is the same question as: *"I keep 86.6% of my paycheck after taxes. I need $10 to take home. How much do I have to earn?"*

- Earn $10 → take home $8.66. Not enough.
- You need to earn **more** than $10, because a chunk is taken off.
- How much more? Undo the tax: **divide** what you need by the share you keep:

```text
$10 ÷ 0.866 = $11.55 to earn
check: $11.55 × 0.866 = $10.00 take-home ✓
```

#### 🧮 If "divide by a decimal" trips you up: flip it into a multiply

Dividing by a decimal is the same trick you already use for fractions: **turn the decimal into a fraction, flip it upside down, and multiply.** (Dividing by a fraction *is* multiplying by its flipped version — that never changes.)

```text
0.866  is just  866/1000

$10 ÷ 0.866
  = $10 ÷ (866/1000)
  = $10 × (1000/866)     ← flip the fraction, turn ÷ into ×
  = $10000 / 866
  = $11.55  ✓
```

So the shortcut is: **"divide by 0.866" = "multiply by 1000/866" ≈ "multiply by 1.155."** Multiplying by 1.155 just means "make it about 15% bigger" — which is the same 15% harder the drone has to push. Same answer, no long division by a decimal.

The drone is exactly the same, with "push" in place of "paycheck" and "up piece" in place of "take-home":

```text
push × 0.866 = 10 N up          ← "after the tilt takes its share, 10 N must be left"
push         = 10 ÷ 0.866       ← undo the × by dividing (like undoing the tax)
             = 11.55 N
check: 11.55 × 0.866 = 10.0 N up ✓
```

So the drone must push **11.55 N instead of 10**, about **15% harder**.

#### The one algebra move

The only move in there is: **"if something × 0.866 = 10, then something = 10 ÷ 0.866."** Multiplying and dividing undo each other. That's all.

#### Now the letters

Swap the numbers for names:

```text
10 N          →  H       (the push you'd need if level: "Hover")
0.866         →  cos(θ)  (the share that still points up)
the new push  →  T       ("Thrust" you actually need)
```

Same two lines as before:

```text
T × cos(θ) = H          "after the tilt takes its share, H must be left"
T          = H ÷ cos(θ) "undo the × by dividing"
```

And `H ÷ cos(θ)` is the same thing as `H × (1 ÷ cos(θ))`. Dividing by 0.866 is the same as multiplying by `1 ÷ 0.866 = 1.155`. The code likes the multiply form, because it gives one number to scale the throttle by:

```text
tiltFactor = 1 ÷ cos(θ)       at 30°: 1 ÷ 0.866 = 1.155   → "push 15.5% harder"
```

So: **multiply the level throttle by `1/cos(θ)`**. That multiplier is `tiltFactor`.

```text
tilt     cos      1/cos (tiltFactor)    meaning
  0°     1.000      1.00                no boost needed
 10°     0.985      1.02                +2%
 30°     0.866      1.15                +15%
 45°     0.707      1.41                +41%
 60°     0.500      2.00                double it
 80°     0.174      5.76                (absurd)
 89°     0.017     57.3                 (impossible)
```

### Piece 2 — why *two* cosines, multiplied

A drone can tilt two ways at once: **roll** (lean left/right) and **pitch** (lean forward/back). Each one shrinks the up piece by its own cosine, and they stack like discounts in a store:

```text
"30% off, then an extra 10% off" is NOT 40% off.
It's 0.70 × 0.90 = 0.63 of the price → 37% off.
```

Same here. Pitch leaves `cos(pitch)` of the lift pointing up. Roll then takes *that* and leaves `cos(roll)` of it:

```text
fraction still pointing up = cos(roll) × cos(pitch)
tiltFactor                 = 1 / (cos(roll) × cos(pitch))
```

With roll 20° and pitch 10°: `0.940 × 0.985 = 0.926`, so 92.6% of the lift still points up, and `tiltFactor = 1 / 0.926 = 1.08`, an 8% boost.

### Piece 3 — the two clamps

`constrain(x, lo, hi)` means "force `x` to stay between `lo` and `hi`."

- **Floor of 1.0:** mathematically `1/cos` is never below 1, but a glitchy sensor reading could produce something odd. The floor guarantees this code can only ever *add* throttle, never cut it.
- **Ceiling of 2.0:** look at the table. Near 90° the factor explodes toward infinity. A drone tilted that far is nearly on its side. That's an emergency, not something to "compensate" for. So the boost stops at 2.0, which is exactly 60°, the number in the code comment.

### Piece 4 — applying it

```cpp
out.baseThrottle = constrain(out.baseThrottle * tiltFactor, 0.0f, 1.0f);
```

Multiply the throttle by the boost, then clamp to 0–1 because a motor can't go past 100%.

### 🔢 Numbers

Level hover throttle 0.50, roll 20°, pitch 10°:

```text
0.50 × 1.08 = 0.54     → the drone quietly goes from 50% to 54% as it leans
```

At level, `tiltFactor = 1.0` and nothing changes.

### ✍️ Your turn

Hover throttle 0.60, roll 0°, pitch 45°. What throttle does the code command?

> **Answer:** `tiltFactor = 1 / (cos 0° × cos 45°) = 1 / (1 × 0.707) = 1.41`. That's inside [1, 2], so no clamp. `0.60 × 1.41 = 0.85`. Inside [0, 1] ✓. **0.85**.

**Takeaway:** *Divide by `cos(roll)·cos(pitch)` to give back exactly the lift the tilt stole, capped so it can't run away.*

---

## Lesson 1.6 — Running it backward: tangent as "steepness"

Everything so far went one direction:

```text
angle ──── cos / sin ────► pieces      "I'm leaning 30°. How much goes up?"
```

Navigation needs the opposite:

```text
pieces ──── ??? ────► angle            "The waypoint is 4 m north and 3 m east. Which way?"
```

GPS never says "the waypoint is at 37°." It gives two distances. The drone has to work out the angle. This lesson and the next two build that `???`.

### 🧸 ELI5

A wheelchair ramp has a sign: *"rises 1 foot for every 2 feet forward."* That describes how **steep** it is using two lengths and no angle. But the ramp obviously has an angle; you could measure it with a protractor.

```text
                         ___●
                    ___╱    │
               ___╱         │ rise = 1
          ___╱  θ           │
         ●──────────────────┘
               run = 2
```

So steepness has two languages:

```text
as a RATIO:   rise ÷ run = 1 ÷ 2 = 0.5
as an ANGLE:  about 26.6°
```

**Tangent** translates angle → ratio. **Arctangent** translates ratio → angle.

> **Calc flashback:** "rise over run" is **slope**, the thing you took derivatives of in Calc 1. `tan(θ)` is literally the slope of a line tilted at angle θ.

### 🖼️ Picture

You already have two ratios. Tangent is the third, and it's the only one that **ignores the long slanted side**:

```text
cos(θ) = along ÷ slanted
sin(θ) = sticks-out ÷ slanted
tan(θ) = sticks-out ÷ along       ← rise over run; no slanted side needed
```

That's exactly why navigation wants it: GPS gives the two straight pieces (north and east), not the slanted distance.

(Useful check: `tan(θ) = sin(θ) / cos(θ)`. The slanted sides cancel.)

### 🔢 Numbers

```text
angle     tan = rise ÷ run      looks like
  0°        0                   flat
 26.6°      0.5                 the wheelchair ramp
 45°        1                   perfect diagonal: rise = run
 60°        1.73                steep
 80°        5.67                very steep
 90°        infinite            a wall: run is 0, can't divide by it
```

Remember that last row. It causes trouble in Lesson 1.8.

**Takeaway:** *`tan` turns an angle into a steepness ratio (rise ÷ run).*

---

## Lesson 1.7 — `atan`: the undo button

### 🧸 ELI5

`atan` (also written `arctan` or `tan⁻¹`) is tangent run backward, like converting °F back to °C.

```text
tan(45°) = 1       "a 45° slope rises 1 per 1"
atan(1)  = 45°     "a slope of 1 per 1 is 45°"
```

### 🔢 Numbers

The ramp: `atan(1 ÷ 2) = atan(0.5) ≈ 26.6°`.

The waypoint, **3 m east** and **4 m north**. If we measure the angle **up from east** (classroom style):

```text
   north
     ▲
   4 ┤         ● waypoint
     │        ╱│
   3 ┤       ╱ │
     │      ╱  │ 4 m north (sticks out from the east line)
   2 ┤     ╱   │
     │    ╱    │
   1 ┤   ╱     │
     │  ╱ θ    │
     ●─────────┘──────► east
   drone   3 m east (along the east line)

tan(θ) = sticks-out ÷ along = 4 ÷ 3 = 1.333
θ      = atan(1.333) ≈ 53.1°      "53° up from due east"
```

Sanity check: 4 north is more than 3 east, so the arrow leans more toward north than east. That means more than 45° from east. ✓

### ✍️ Your turn

A drone is 10 m east and 10 m north of its launch pad. What angle, measured from east, is it at?

> **Answer:** `atan(10 ÷ 10) = atan(1) = 45°`. Equal pieces means perfect diagonal. ✓

**Takeaway:** *`atan(rise ÷ run)` gives back the angle.* But it has a blind spot, which is next.

---

## Lesson 1.8 — `atan2`: the version that can't get lost

### 🧸 ELI5

I hand you a note: *"walk 1 step north for every 1 step east."* You go northeast.

Another note: *"walk −1 north for every −1 east."* That's 1 south for every 1 west. You go **southwest**, the opposite direction.

Now suppose you only hand those notes to `atan` *after dividing*:

```text
northeast:    1 ÷  1 = 1   → atan(1) = 45°
southwest:   −1 ÷ −1 = 1   → atan(1) = 45°    ✗ WRONG, opposite direction
```

The two minus signs cancelled. By the time `atan` sees `1`, the fact that "both were negative" is **gone**. It's like being told a game ended "3 to 4" without being told which team scored which.

### 🖼️ Picture

Four diagonal directions, written as `(east, north)`, and what plain `atan(north ÷ east)` thinks each one is (angle from east):

```text
                    north (+)
                      │
   (−1,+1)       ↖    │    ↗       (+1,+1)
   ratio −1           │            ratio +1
   atan says −45° ✗   │            atan says 45° ✓
                      │
  west ───────────────●─────────────── east (+)
                      │
   (−1,−1)       ↙    │    ↘       (+1,−1)
   ratio +1           │            ratio −1
   atan says 45° ✗    │            atan says −45° ✓
                      │
                    south
```

Plain `atan` only ever answers between −90° and +90°. It can only "see" the east half of the map, and it silently mirrors everything on the west side. Plus, a target due north has east = 0, so the ratio divides by zero.

### The fix: don't divide first

```text
atan(y ÷ x)      you divide, the signs are lost, then it guesses
atan2(y, x)      it gets both numbers, looks at both signs, then answers
```

That's the whole difference. The "2" means "takes two arguments." Because `atan2` can see each sign, it knows which quarter of the map you're in, and it answers with the full circle, −180° to +180°:

```text
(y, x)       direction       atan2       plain atan(y÷x)
( 1,  1)     up-right          45°         45°
( 1, −1)     up-left          135°        −45°   ✗
(−1, −1)     down-left       −135°         45°   ✗
(−1,  1)     down-right       −45°        −45°
( 1,  0)     straight up       90°        divide by zero ✗
```

> **Mental model:** `atan(ratio)` is being told *how steep*. `atan2(y, x)` is being handed *the actual arrow*. With the arrow in hand, you can't get the direction wrong.

### Which argument goes first?

`atan2` takes `y` first, then `x`. Don't memorize that. Use the Lesson 1.2 rule instead. `atan2` undoes the cos/sin split, so:

```text
atan2( sticks-out piece , along piece )
atan2(    sine piece    , cosine piece )
```

- The **second** argument is the line you measure **from**.
- The **first** argument is the direction you turn **toward**.

Classroom: measure from x, turning toward y → `atan2(y, x)`. ✓

### Turning it into a compass bearing

A compass is not a classroom graph:

```text
   MATH angle                          COMPASS bearing
   start at EAST, turn counter-        start at NORTH, turn
   clockwise toward north              clockwise toward east

          90°                                 0°
           │                                  │
  180° ────●──── 0°                  270° ────●──── 90°
           │                                  │
         −90°                               180°
```

With the rule, the fix is free. A compass measures **from north** (the along piece, goes second), turning **toward east** (the sticks-out piece, goes first):

```text
bearing = atan2( east , north )
```

### 🔢 Numbers

Waypoint 3 east, 4 north:

```text
atan2(3, 4) ≈ 36.9°      "37° clockwise from north"   ✓ (closer to north than east, so under 45°)
```

Waypoint 3 **west**, 4 **south** (east = −3, north = −4):

```text
atan2(−3, −4) ≈ −143.1°
```

A negative bearing just means "143° **counter**-clockwise from north." Compasses want 0–360°, so add 360:

```text
−143.1° + 360° = 216.9°       between south (180°) and west (270°)   ✓ southwest
```

### 💻 In the code

`gpsBearing()` in [`NavMath.hpp`](src/flight/services/NavMath.hpp):

```cpp
inline float gpsBearing(double lat1, double lon1, double lat2, double lon2) {
  float dLon = radians(lon2 - lon1);
  float y    = sin(dLon) * cos(radians(lat2));
  float x    = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  return fmod(degrees(atan2(y, x)) + 360.0f, 360.0f);
}
```

Don't decode the `y` and `x` lines. All you need to know is:

```text
y  =  "how far EAST"   (with a correction for Earth being round)
x  =  "how far NORTH"  (with a correction for Earth being round)
```

Longitude is the east/west coordinate, so the line built from `dLon` is the east one. Now read the last line inside-out:

```text
atan2(y, x)          = atan2(east, north)   angle FROM north TOWARD east, in radians
degrees( … )         radians → degrees      now −180° … +180°
… + 360.0f           the "add 360" fix      now 180° … 540°, never negative
fmod( … , 360.0f)    remainder after ÷ 360  wraps 405° back to 45°; final 0° … 360°
```

Why `fmod`? Adding 360 fixes negatives, but pushes positives past 360 (45° becomes 405°). `fmod(value, 360)` is "the remainder after dividing by 360." It turns 405 back into 45 and leaves 216.9 alone. One line handles both cases with no `if`.

```text
atan2( 3,  4) =   36.9° → +360 = 396.9° → fmod →  36.9°  ✓
atan2(−3, −4) = −143.1° → +360 = 216.9° → fmod → 216.9°  ✓
```

### The round trip

`NavMath.hpp` has both directions side by side:

```text
gpsBearing()           pieces ──atan2──► angle
bearingToNorthEast()   angle  ──cos/sin──► pieces
```

Prove they undo each other, starting from 3 east, 4 north:

```text
distance = √(3² + 4²) = 5 m
bearing  = atan2(3, 4) = 36.9°
north    = 5 × cos(36.9°) = 5 × 0.8 = 4 m   ✓
east     = 5 × sin(36.9°) = 5 × 0.6 = 3 m   ✓
```

### ✍️ Your turn

A waypoint is **5 m west** and **5 m north**.

1. What are `east` and `north`, with signs?
2. What would `atan(north ÷ east)` say, and why is it wrong?
3. What does `atan2(east, north)` give?
4. What does the `+360` / `fmod` turn that into?
5. Does it match "northwest"?

> **Answer:**
> 1. east = −5, north = +5
> 2. `atan(5 ÷ −5) = atan(−1) = −45°`. It lost the signs. It can't tell northwest from southeast, since both have ratio −1.
> 3. `atan2(−5, 5) = −45°`, meaning 45° counter-clockwise from north. Correct.
> 4. −45 + 360 = 315, and fmod(315, 360) = **315°**
> 5. Yes. West is 270°, north is 360°, and halfway is 315°. ✓

**Takeaway:** *`atan2(sticks-out, along)` turns two pieces back into an angle without losing the direction. For a compass: `atan2(east, north)`, then wrap into 0–360.*

---

## Part 1 checkpoint

Explain each of these out loud, without looking:

- [ ] what `cos` and `sin` are (the two shadows)
- [ ] the "along vs. sticks out" rule, and why cosine is the *up* piece for a tilted drone
- [ ] why code multiplies degrees by 0.01745 before calling `cosf`
- [ ] why `tiltFactor = 1/(cos(roll)·cos(pitch))`, and what each clamp protects against
- [ ] what `tan` means (steepness) and what `atan` undoes
- [ ] why `atan(north/east)` can't tell northeast from southwest
- [ ] why the compass version is `atan2(east, north)`, and what `+360`/`fmod` do

If two or more feel shaky, re-read the ELI5s for those lessons before moving on.

---

# Part 2 — Vectors: arrows as lists of numbers

## Lesson 2.1 — A vector is a treasure-map instruction

### 🧸 ELI5

A treasure map says: *"From the big oak, walk 3 steps east, then 4 steps north."* That instruction is a **vector**. It has a **direction** (roughly north-east) and a **size** (how far). Writing it as numbers is just shorthand:

```text
(3, 4)     means     3 east, 4 north
```

Compare that with a plain number like "5 meters." That's a **scalar**: a size with no direction. "The waypoint is 5 m away" is a scalar. "The waypoint is 3 m east and 4 m north" is a vector. The vector tells you strictly more.

### 🖼️ Picture

```text
   north
     ▲
   4 ┤         ● (3, 4)
     │        ↗
     │      ↗
     │    ↗
     │  ↗
     ●──────────► east
   start  3
```

A vector is an arrow. The numbers are "how far along each axis the arrow's tip is from its tail."

### 🔢 Numbers

The drone uses vectors everywhere, always in this order in this course: **(east, north)**.

```text
where I am, relative to launch         (10, 20)     10 m east, 20 m north
where the waypoint is                  (40, 60)
the wind pushing me                    (−2, 0)      2 m/s toward the west
```

A 3D vector just adds a third number: (east, north, up). QuadSim uses 3D. Most of the navigation math uses 2D, because "up" is handled separately by the altitude controller.

**Takeaway:** *A vector is a list of numbers that together describe an arrow: how far along each direction.*

---

## Lesson 2.2 — Length: how long is the arrow?

### 🧸 ELI5

You walked 3 east and 4 north. How far are you from the oak, **as the crow flies**? Not 7. The straight-line shortcut is shorter than walking the two legs. That's what the triangle's long side is.

### 🔢 Numbers

Pythagoras, which you already used in Part 1:

```text
length of (east, north) = √(east² + north²)

length of (3, 4)   = √(9 + 16)   = √25  = 5
length of (6, 8)   = √(36 + 64)  = √100 = 10
length of (−3, −4) = √(9 + 16)   = 5         ← squaring kills the minus signs,
                                                so direction doesn't affect length
```

Math books write length as `|v|` or `‖v‖` and call it the **magnitude**. Same thing.

### 💻 In the code

[`NavMath.hpp`](src/flight/services/NavMath.hpp), inside `lineFollowNorthEast()`:

```cpp
float segLen = sqrt(segE * segE + segN * segN);
```

That's `√(east² + north²)`: the length of the leg between two waypoints. And in the sim app's [`FlightLog.hpp`](src/apps/sim/FlightLog.hpp), the flight log computes distance from home the same way:

```cpp
float dist = sqrtf(north * north + east * east);
```

**Takeaway:** *Length = `√(east² + north²)`. Pythagoras, every time.*

---

## Lesson 2.3 — Adding and subtracting: "from me to there"

### 🧸 ELI5

**Adding** is following two instructions in a row. "3 east, 4 north" then "1 east, 2 north" gets you to "4 east, 6 north." Just add each column.

**Subtracting** answers the most important question in navigation: *"how do I get from where I am to where I want to be?"* If you know where **you** are and where the **target** is (both measured from the same starting point), then:

```text
arrow from me to target  =  target − me
```

Think "destination minus origin." It's like asking how many floors to ride in an elevator: `destination floor − current floor`.

### 🖼️ Picture

```text
   north
     ▲
  60 ┤                    ● target (40, 60)
     │                  ↗
     │   target − me  ↗
     │  = (30, 40)  ↗
  20 ┤          ● me (10, 20)
     │        ↗
     │      ↗  "me"
     │    ↗
     ●──────────────────────► east
   launch
```

### 🔢 Numbers

```text
me     = (10, 20)
target = (40, 60)
target − me = (40 − 10, 60 − 20) = (30, 40)

"Go 30 m east and 40 m north."  Distance = √(30² + 40²) = 50 m.
```

Get the order backward (`me − target`) and you get (−30, −40): an arrow pointing *away* from the target. That's one of the most common sign bugs in drone code.

### 💻 In the code

This "target minus me" pattern is the **error** that every controller works on (Part 4). In `lineFollowNorthEast()`:

```cpp
eastM  = carrotE - posE;     // "vector from us to the carrot" = carrot − me
northM = carrotN - posN;
```

And inside every PID:

```cpp
float error = target - actual;
```

Same idea, one dimension.

### ✍️ Your turn

You're at (50, 10). The waypoint is at (20, 50). What's the arrow from you to the waypoint, and how far is it?

> **Answer:** (20 − 50, 50 − 10) = **(−30, 40)**, i.e. 30 m *west* and 40 m north. Length = √(900 + 1600) = **50 m**.

**Takeaway:** *The arrow from me to the target is `target − me`. Order matters.*

---

## Lesson 2.4 — Scaling and unit vectors: direction without distance

### 🧸 ELI5

Multiplying a vector by a number just stretches or shrinks the arrow: "3 east, 4 north" times 2 is "6 east, 8 north," same direction, twice as far.

Sometimes you want **only the direction**: "which way is the next waypoint?" and not "how far?" So you shrink the arrow to length exactly **1**. That's a **unit vector**. It's like a compass needle: it points, but it doesn't tell you distance.

### 🔢 Numbers

To make a unit vector, divide each piece by the length:

```text
v = (3, 4)          length 5
v ÷ 5 = (0.6, 0.8)  length √(0.36 + 0.64) = √1 = 1   ✓
```

Notice: `(0.6, 0.8)` is `(sin, cos)` of the 36.9° bearing from Lesson 1.8. A unit vector's pieces **are** a sine and a cosine. That's the unit-stick from Lesson 1.1 again.

Once you have a unit vector `u`, you can build a point **any distance** along that direction just by scaling:

```text
18 m along u = 18 × (0.6, 0.8) = (10.8, 14.4)
```

This is called **normalizing**. Watch out for one trap: if the arrow has length 0, you'd divide by zero.

### 💻 In the code

In `lineFollowNorthEast()`:

```cpp
float segLen = sqrt(segE * segE + segN * segN);
if (segLen < 1.0f) {                    // tiny leg: avoid dividing by ~0
  ...
  return;
}
float uE = segE / segLen, uN = segN / segLen;         // unit vector along the leg
...
float carrotE = uE * carrot, carrotN = uN * carrot;    // a point `carrot` meters along it
```

That's all three ideas: compute the length, guard against dividing by zero, divide to get a unit vector, then scale it to place a point.

**Takeaway:** *Unit vector = arrow ÷ its own length. Pure direction, length 1. Scale it to walk any distance that way.*

---

## Lesson 2.5 — The dot product: "how much do two arrows agree?"

### 🧸 ELI5

You and a friend are pushing a stalled car. If you both push in the same direction, your efforts fully **agree**. If your friend pushes sideways, their effort does nothing to help you (zero agreement). If they push from the front, they're **fighting** you (negative agreement).

The dot product is a single number that measures that agreement.

### 🔢 Numbers

The recipe: multiply matching pieces, then add.

```text
a · b = (a_east × b_east) + (a_north × b_north)
```

```text
same direction:     (1, 0) · (1, 0)  = 1 + 0 =  1     fully agree
perpendicular:      (1, 0) · (0, 1)  = 0 + 0 =  0     no agreement
opposite:           (1, 0) · (−1, 0) = −1 + 0 = −1    fully disagree
somewhere between:  (1, 0) · (0.6, 0.8) = 0.6          mostly agree
```

The sign is the quick read: **positive** means roughly the same way, **zero** means perpendicular, **negative** means roughly opposite.

There's a second way to write the dot product, and it connects straight back to trig:

```text
a · b = |a| × |b| × cos(angle between them)
```

For two **unit** vectors, both lengths are 1, so the dot product **is** the cosine of the angle between them. Check: `(1, 0) · (0.6, 0.8) = 0.6`. The arrow `(0.6, 0.8)` sits 53.1° up from east (Lesson 1.7 worked that out for 3-and-4), and `cos(53.1°) = 0.6` ✓.

Most math classes stop here: two formulas, "they're equal, move on." But *why* would multiplying matching pieces and adding have anything to do with a cosine? The rest of this lesson is the backstory.

---

### 📜 The backstory, part 1: the question people kept asking

Before canals had engines, barges were pulled by a horse walking along the bank, with a long rope:

```text
   bank    🐎 ─────────────╮  the horse pulls ALONG THE ROPE (at an angle)
              ╲            │
               ╲  rope     │
                ╲          │
   canal         ▭ barge ──────────────►  but the barge can only move ALONG THE CANAL
```

The horse pulls at an angle, but the barge can only go down the canal. So how much of the horse's pull actually moves the barge?

You already know the answer from Lesson 1.1: it's the **shadow** of the pull along the canal.

```text
useful pull = (how hard the horse pulls) × cos(angle between rope and canal)
```

Physicists in the 1800s needed this constantly. "How much of this force helps this motion?" is the definition of **work** (energy), and it shows up everywhere: steam engines, planets, electricity. It's the same question as the stalled car in the ELI5, and the same question as "how much of my thrust points up?" in Lesson 1.4.

### 📜 Part 2: the annoying part

That formula needs **the angle between** the two arrows. Usually you don't have the angle. You have **coordinates**: "the rope pulls (3, 4), the canal runs (0.8, 0.6)."

To use the cosine formula you'd have to:

```text
1. find the rope's angle        atan2 → 53.13°
2. find the canal's angle       atan2 → 36.87°
3. subtract                     16.26°
4. take the cosine              cos(16.26°) = 0.96
5. multiply by the rope's pull  5 × 0.96 = 4.8
```

That's three trig calculations, done by hand with printed tables in the 1800s. Slow, and easy to get wrong.

### 📜 Part 3: the shortcut (and where the name "dot" comes from)

It turns out there's a shortcut that **never touches an angle**:

```text
(3, 4) · (0.8, 0.6) = 3×0.8 + 4×0.6 = 2.4 + 2.4 = 4.8      same answer, no trig at all
```

The shortcut showed up almost by accident. In 1843 the Irish mathematician William Rowan Hamilton invented **quaternions**: the same quaternions QuadSim uses to track orientation (Part 10). When he multiplied two arrows together as quaternions, the answer came out as two parts glued together: a plain number, and a new arrow. The plain number was exactly this multiply-and-add (with a minus sign in front).

About 40 years later, Josiah Willard Gibbs (at Yale) and Oliver Heaviside (in England) were working with Maxwell's equations for electricity and magnetism. They found quaternions clumsy for physics, so they pulled the two glued parts apart and gave each one its own name. Gibbs wrote the first one with a dot between the arrows, `a · b`, and the name stuck: the **dot product**. The other part became the **cross product**. (Gibbs's notes on this became the vector notation still taught today.)

So the dot product wasn't really *invented* to be a formula. It was **discovered** to be a fast way to answer an old question ("how much does this agree with that?") using only multiplication and addition.

### 📜 Part 4: why the shortcut works

This is the part classes skip. It takes four small steps, and each one is something you can picture.

**Setup.** We want to know how far arrow **a** = (3, 4) reaches **along a road** whose direction is the unit arrow **u** = (0.8, 0.6). "How far along the road" is the shadow, so we want the shadow of **a** on the road.

**Step 1 — Split the trip into two legs.** Arrow **a** is the same as walking **3 steps east, then 4 steps north**:

```text
          ●  end of a
          │
          │ 4 north
          │
  ●───────┘
 start  3 east
```

**Step 2 — Progress along the road adds up, leg by leg.** Picture walking those two legs next to a straight road. The east leg moves you some distance along the road, and the north leg moves you some more. Your total progress is just the two added together:

```text
total progress along the road = progress from the east leg + progress from the north leg
```

(Shadows add. If you walk a crooked path, your shadow's total movement is the sum of each step's movement.)

**Step 3 — How much progress does ONE step east make?** This is the clever step.

One step east is a length-1 arrow pointing east. The road is a length-1 arrow pointing along **u**. Put their tails together. There's some angle θ between them.

```text
                    ● u (length 1)
                   ╱│
                  ╱ │
                 ╱ θ│
  east step  ●──────●──►  (length 1)
```

- The shadow of the **east step** on the road is `cos θ`.
- The shadow of the **road arrow** on the east line is also `cos θ`.

Same two sticks, same angle, same length, so they cast the same shadow on each other. It's symmetric.

And the shadow of **u** on the east line is easy: it's just **u's east number**, 0.8. (The east number of an arrow *is* how far it reaches east. That's what the coordinates mean.)

So: **one step east makes 0.8 of progress along the road.** By the same argument, **one step north makes 0.6** of progress (u's north number).

**Step 4 — Add it up.**

```text
3 steps east  × 0.8 progress each = 2.4
4 steps north × 0.6 progress each = 2.4
                              total = 4.8      ← that's a_east×u_east + a_north×u_north
```

**That's the recipe.** "Multiply matching pieces and add" literally means: *for each direction, (how far I go that way) × (how much that way helps the road), and add those up.*

If the second arrow isn't length 1 (say it's **b** with length 10), every "progress" number just gets 10 times bigger. That's why the general version has both lengths in it: `a · b = |a| × |b| × cos θ`.

(Your Calc 3 textbook probably proved this with the **law of cosines** instead. That's a valid proof, but it hides the "legs of a trip" picture above.)

### 📜 Part 5: why the sign means what it means

The sign comes straight from the cosine, which comes straight from the shadow:

```text
angle between    cos θ     the shadow...                          dot product
   0°             1        is the whole arrow, pointing forward   biggest positive
  under 90°      +         points forward (helps)                 positive
   90°            0        is a single point (no help at all)     zero
  over 90°       −         points BACKWARD (works against you)    negative
  180°           −1        is the whole arrow, pointing backward  biggest negative
```

A negative dot product is a shadow that falls **behind** you: the horse walking the wrong way down the bank, dragging the barge backward.

### 📜 Part 6: why your drone loves it

A computer can do "two multiplies and an add" in a few nanoseconds, with no `atan2` and no `cos` calls. That's why the dot product shows up all over the firmware, sometimes in disguise:

```text
where                               the dot product                          the question it answers
lineFollowNorthEast() (Lesson 7.2)  posE*uE + posN*uN                        how far along the leg am I?
northEastToForwardRight() (10.1)    north*cos(h) + east*sin(h)               how much of the error is ahead of my nose?
tiltFactor (Lesson 1.5)             cos(roll) × cos(pitch)                   how much of my "up" agrees with the world's up?
```

That last one is a surprise. `cos(roll) × cos(pitch)` is the dot product of the **drone's up arrow** with the **world's up arrow** (0, 0, 1). The world's up arrow has only an up piece, so the dot product just picks out the up piece of the drone's arrow. That up piece works out to `cos(roll) × cos(pitch)`. It's the same horse-and-barge question: "how much of the thrust helps hold the drone up?"

### ✍️ Your turn

a = (2, 3), b = (3, −2). Compute a · b. What does it tell you?

> **Answer:** (2×3) + (3×−2) = 6 − 6 = **0**. They're perpendicular.

**Takeaway:** *Dot product = multiply matching pieces and add. It measures how much two arrows point the same way.*

---

## Lesson 2.6 — Projection: the shadow on a line

### 🧸 ELI5

Back to shadows. You're walking a straight road, and you've wandered off into the field beside it. Someone asks, *"how far along the road have you gotten?"* Not how far from the start in a straight line, but how far **along the road**. Drop a line straight from you onto the road. Where it lands is your "progress along the road."

That's a **projection**, and the dot product computes it in one step.

### 🖼️ Picture

```text
                     ● me (wandered off the road)
                     ┆
                     ┆  ← drop straight down to the road
                     ┆
  start ●────────────┼──────────────────────► road direction (unit vector u)
        │◄── proj ──►│
          "how far along the road I am"
```

### 🔢 Numbers

If `u` is a **unit** vector along the road and `p` is your position (measured from the road's start):

```text
progress along the road = p · u
```

Example: the road points east, `u = (1, 0)`. You're at `p = (30, 10)`, 30 m east and 10 m north (off the road).

```text
p · u = 30×1 + 10×0 = 30    → you're 30 m along the road   ✓ obviously
```

A harder one: the road runs at bearing 36.9°, `u = (0.6, 0.8)`. You're at `p = (40, 20)`.

```text
p · u = 40×0.6 + 20×0.8 = 24 + 16 = 40    → 40 m along the road
```

The piece of you that **agrees** with the road direction is your progress. The rest is how far off the road you are.

**Why must `u` be a unit vector?** If `u` had length 5, the answer would come out 5 times too big. Normalizing first (Lesson 2.4) is what makes the dot product mean "meters along the road."

### 💻 In the code

In `lineFollowNorthEast()`:

```cpp
float proj = posE * uE + posN * uN;                    // how far along the leg we are
if (proj < 0.0f) proj = 0.0f;
```

That's exactly `p · u`. If the answer is negative, you're *behind* the start of the leg, so the code clamps it to 0. Part 7 builds the rest of this function.

### ✍️ Your turn

Road direction `u = (0, 1)` (due north). You're at `(15, 25)`. How far along the road are you? How far off it?

> **Answer:** `p · u = 15×0 + 25×1 = 25` m along. Off the road: 15 m, the east piece that doesn't agree with north.

**Takeaway:** *Projection = `position · unit_direction`. It answers "how far along this line am I?"*

---

## Part 2 checkpoint

- [ ] what a vector is (a treasure-map instruction), vs. a scalar
- [ ] length = `√(east² + north²)`
- [ ] "from me to the target" = `target − me`, and what happens if you flip it
- [ ] what a unit vector is and how to make one (and the divide-by-zero trap)
- [ ] the dot product recipe, and what positive / zero / negative mean
- [ ] projection = "how far along the road," and why `u` must be a unit vector

---

# Part 3 — Calculus refresher: slope and area

You did three semesters of calculus. The drone uses exactly **two** ideas from it, and it uses them in the simplest possible form: no symbols to integrate, just arithmetic done 200 times a second. This part is a refresher, not a course.

## Lesson 3.1 — The derivative is a speedometer

### 🧸 ELI5

Your car's odometer says how far you've gone. Your **speedometer** says how *fast that number is changing* right now. The derivative is the speedometer for any number: *"how fast is this changing?"*

### 🖼️ Picture

On a graph, "how fast is it changing" is the **slope**: rise over run, the tangent from Lesson 1.6.

```text
 altitude
   (ft)
    12 ┤                ●
       │              ╱   ← steep: altitude changing fast
    10 ┤           ●╱
       │        ╱
     8 ┤  ● ─ ●         ← flat: not changing
       │
       └──┬───┬───┬───┬───► time (s)
          0   1   2   3
```

### 🔢 Numbers

In Calc 1 you took a limit to make the run infinitely small. A computer can't do that. It just uses **the last two readings**:

```text
derivative ≈ (new − old) ÷ dt
```

Altitude was 10.00 ft, and 5 ms later (dt = 0.005 s) it's 10.01 ft:

```text
(10.01 − 10.00) ÷ 0.005 = 0.01 ÷ 0.005 = 2 ft/s     → climbing at 2 ft/s
```

💡 **That "÷ 0.005" is the flip trick from Lesson 1.5.** Dividing by `dt = 0.005` is the same as multiplying by `1 ÷ 0.005 = 200`:

```text
0.01 ÷ 0.005  =  0.01 × (1/0.005)  =  0.01 × 200  =  2   ✓
```

So every `÷ dt` in this course is secretly a `× 200` (whenever `dt = 0.005`). That's worth remembering: dividing by a tiny number multiplies by a *big* one, which is exactly why derivatives can spike so hard.

Units: ft ÷ s = ft/s. A derivative always has "per second" glued on.

### 💻 In the code

In [`PID.hpp`](src/flight/services/PID.hpp):

```cpp
float derivative = (error - _lastError) / dt;
...
_lastError = error;          // remember this one for next time
```

That's `(new − old) ÷ dt`, with the "old" value saved in a variable between calls.

**Takeaway:** *Derivative = `(new − old) ÷ dt` = how fast something is changing.*

---

## Lesson 3.2 — The integral is a running total

### 🧸 ELI5

A bathtub with the tap running. The tap's flow rate (liters per second) is what's happening *right now*. The water in the tub is **everything that's flowed in so far**: a running total. The integral is that running total.

It works in reverse too. If you know your speed every second, adding up `speed × time` for every little slice gives you total distance. That's the odometer, built from the speedometer.

### 🖼️ Picture

In Calc 2 the integral was the **area under the curve**. Here's why: each thin slice is `height × width` = `value × dt`, and adding all the slices gives the area.

```text
 value
       │   ┌─┐
       │ ┌─┤ ├─┐
       │ │ │ │ ├─┐
       │ │ │ │ │ │      each bar: value × dt
       └─┴─┴─┴─┴─┴──► time
         └── total area = running sum of all the bars
```

### 🔢 Numbers

The computer version is a single line, run every tick:

```text
total += value × dt
```

Speed is a steady 2 m/s, dt = 0.5 s:

```text
start:    total = 0
tick 1:   total = 0   + 2 × 0.5 = 1 m
tick 2:   total = 1   + 2 × 0.5 = 2 m
tick 3:   total = 2   + 2 × 0.5 = 3 m     → after 1.5 s at 2 m/s, you've gone 3 m ✓
```

### 💻 In the code

In [`PID.hpp`](src/flight/services/PID.hpp):

```cpp
_integral += error * dt;
```

And in [`QuadSim.h`](lib/QuadSim/src/QuadSim.h), speed and position are both running totals:

```cpp
s_.vel[0] += ax * h;           // speed    = running total of acceleration
s_.pos[0] += s_.vel[0] * h;    // position = running total of speed
```

**Takeaway:** *Integral = `total += value × dt` = a running total over time.*

---

## Lesson 3.3 — The two are opposites

### 🧸 ELI5

The speedometer (derivative) and the odometer (integral) are two views of the same trip. Given one, you can rebuild the other. That's the Fundamental Theorem of Calculus from Calc 1, and the drone uses it constantly:

```text
                derivative (÷ dt)              derivative (÷ dt)
   position  ─────────────────────►  velocity  ─────────────────────►  acceleration
   (m)       ◄─────────────────────  (m/s)     ◄─────────────────────  (m/s²)
                integral (× dt, add up)        integral (× dt, add up)
```

Going right: "how fast is it changing?" Going left: "add it all up."

### ✍️ Your turn

A drone's altitude error readings, every 0.1 s, are: 4, 4, 3, 2. (dt = 0.1)

1. What's the derivative between the last two readings?
2. What's the running total (integral) after all four, starting from 0?

> **Answer:**
> 1. (2 − 3) ÷ 0.1 = **−10** per second (flip it: `÷ 0.1 = × 10`, so `−1 × 10 = −10`). The error is shrinking fast.
> 2. 4×0.1 + 4×0.1 + 3×0.1 + 2×0.1 = 0.4 + 0.4 + 0.3 + 0.2 = **1.3** (error-seconds).

**Takeaway:** *Derivative and integral are opposite directions on the same chain: position ⇄ velocity ⇄ acceleration.*

---

# Part 4 — PID: "how hard should I push?"

PID is the heart of the drone. Six PIDs run inside [`MotorController.hpp`](src/flight/services/MotorController.hpp): altitude, roll, pitch, yaw, and two for GPS navigation. They're all the same 40-line class, [`PID.hpp`](src/flight/services/PID.hpp). Learn it once and you understand all six.

## Lesson 4.1 — The problem: error

### 🧸 ELI5

You're driving toward a stop line. At every moment you glance at the gap between your bumper and the line and decide how hard to press the pedal. That gap is the **error**.

```text
error = where I want to be − where I am
      = target − actual
```

It's "target minus me" from Lesson 2.3, in one dimension.

### 🔢 Numbers

```text
target altitude 15 ft, actual 10 ft    → error = +5 ft    (too low: push up)
target altitude 15 ft, actual 18 ft    → error = −3 ft    (too high: ease off)
target altitude 15 ft, actual 15 ft    → error =  0       (perfect)
```

The **sign** tells you which way to push. The **size** tells you how far off you are.

A PID's whole job: **turn the error into a push.** For the altitude PID the push is throttle. For roll it's a motor difference. For navigation it's a tilt angle.

### 💻 In the code

```cpp
float compute(float target, float actual, float dt) {
    if (dt <= 0) return 0;
    float error = target - actual;
```

**Takeaway:** *A PID turns `error = target − actual` into a push.*

---

## Lesson 4.2 — P: "how far off am I *right now*?"

### 🧸 ELI5

A **rubber band** tied between you and the target. Far away, it pulls hard. Close, it pulls gently. At the target, it doesn't pull at all.

```text
P push = Kp × error
```

`Kp` ("the P gain") is how stiff the rubber band is.

### 🔢 Numbers

The altitude PID's `Kp` is `0.02` (look for `altitudePID(0.02f, ...)` in the code). Units: throttle per foot of error.

```text
error 5 ft  → P = 0.02 × 5  = 0.10    push throttle up 10%
error 1 ft  → P = 0.02 × 1  = 0.02    push up 2%
error −3 ft → P = 0.02 × −3 = −0.06   ease off 6%
```

### Why P alone isn't enough

**Problem 1: overshoot.** The rubber band only goes slack *at* the target, but by then the drone is moving and has momentum. It sails past, the band pulls back, it sails past the other way... It bounces:

```text
 altitude
       │      ╭─╮
target ┼─────╱───╲────╭─╮──────╭──────
       │    ╱     ╲  ╱   ╲    ╱
       │   ╱       ╰╯     ╰──╯          P only: bounces around the target
       │  ╱
       └────────────────────────► time
```

Stiffer band (bigger `Kp`) = snappier but bouncier. Looser = calmer but sluggish.

**Problem 2: it gives up just short.** Suppose a steady breeze pushes the drone down a little. To fight that, P needs *some* error to create *some* push. So the drone settles slightly below the target, just low enough for the rubber band's pull to match the breeze. It never closes that last gap. This is called **steady-state error**.

D fixes problem 1. I fixes problem 2.

**Takeaway:** *P pushes in proportion to how far off you are now. Alone, it overshoots and stops just short.*

---

## Lesson 4.3 — I: "how long have I been off?"

### 🧸 ELI5

A **grudge**. Every moment you're below the target, you add a little to the grudge. The longer you stay below, the bigger it gets, and the harder you push. Even a tiny error, held long enough, builds a big grudge. So I keeps pushing until the error is **truly zero**. That kills problem 2.

It's the running total from Lesson 3.2:

```text
integral += error × dt          (add up the grudge)
I push    = Ki × integral
```

### 🔢 Numbers

The drone hangs 0.5 ft low because of wind. `Ki = 0.008`. The physics loop runs every `dt = 0.005` s, so after one second there have been 200 ticks:

```text
integral after 1 s = 0.5 ft × 0.005 s × 200 ticks = 0.5 ft·s
I push             = 0.008 × 0.5 = 0.004
after 10 s         = 0.008 × 5.0 = 0.04    the grudge keeps growing until the error is gone
```

### The danger: windup

Imagine the drone is sitting on the ground, armed, with a target of 15 ft, and something is stopping it from climbing. The error is +15 ft the whole time. The grudge grows... and grows... When it's finally free, the grudge is enormous, and the drone rockets way past 15 ft. That's **integral windup**.

The fix is to put a cap on the grudge:

```cpp
_integral += error * dt;
_integral = clamp(_integral, _outMin / _ki, _outMax / _ki);
float i = _ki * _integral;
```

Why divide by `_ki`? Because `i = _ki × _integral`, capping `_integral` at `_outMax / _ki` means `i` can never exceed `_outMax`. For altitude: `0.45 / 0.008 = 56.25` (flip: `÷ 0.008 = × 125`, so `0.45 × 125 = 56.25`), so the I push tops out at exactly 0.45, the PID's output limit.

**Takeaway:** *I adds up the error over time, so even a small, stubborn error eventually gets fixed. Cap it to prevent windup.*

---

## Lesson 4.4 — D: "how fast am I closing in?"

### 🧸 ELI5

**Brakes.** You're driving toward the stop line. You don't wait until the line to brake. You see the gap *closing fast* and ease off early. D watches how fast the error is changing, and pushes against that change.

It's the speedometer from Lesson 3.1:

```text
derivative = (error − lastError) ÷ dt
D push     = Kd × derivative
```

### 🔢 Numbers

The drone is 5 ft low and climbing at 2 ft/s. So each 5 ms tick, the error shrinks by 0.01 ft:

```text
lastError = 5.01, error = 5.00
derivative = (5.00 − 5.01) ÷ 0.005 = −2 ft/s     "the gap is closing at 2 ft/s"
D push     = 0.10 × −2 = −0.20                    "ease off!"
```

(As always, `÷ 0.005 = × 200`: `−0.01 × 200 = −2`.)

The sign is automatic: when the error is **shrinking**, the derivative is negative, so D **subtracts**. When the error is **growing** (the drone is falling away from the target), D adds. It's a shock absorber that resists motion in *either* direction.

### A quirk to know about: derivative kick

`reset()` sets `_lastError = 0`. On the very first tick after a reset, if the error is 5 ft:

```text
derivative = (5 − 0) ÷ 0.005 = 1000       D push = 0.10 × 1000 = 100 (!!)
```

(Flip trick again: `÷ 0.005 = × 200`, and `5 × 200 = 1000`. One step produces a giant number — that's exactly the kick.)

The error didn't really jump from 0 to 5 in 5 ms. The PID just didn't have a real "last" value yet. The output clamp (±0.45) catches it, so the result is one tick of max push. It's harmless here, but now you know why it happens. A common fix is to set `_lastError = error` on the first tick.

**Takeaway:** *D pushes against how fast the error is changing. It's the brakes that stop overshoot.*

---

## Lesson 4.5 — All three together, with real numbers

### 🖼️ Picture

```text
                    ┌──► P = Kp × error               "how far off now"
                    │
error = target − ───┼──► I = Ki × (running total)     "how long I've been off"   ──► sum ──► clamp ──► push
        actual      │
                    └──► D = Kd × (rate of change)    "how fast it's changing"
```

### 🔢 Numbers: one real tick of the altitude PID

Gains from the code: `Kp = 0.02, Ki = 0.008, Kd = 0.10`, output clamp ±0.45.
Situation: target 15 ft, actual 10 ft, climbing at 2 ft/s. The grudge built so far is 10 ft·s. `dt = 0.005`.

```text
error      = 15 − 10                        = 5.00 ft
P          = 0.02 × 5                       = +0.100

integral   = 10 + 5 × 0.005                 = 10.025
I          = 0.008 × 10.025                 = +0.080

derivative = (5.00 − 5.01) ÷ 0.005          = −2 ft/s
D          = 0.10 × −2                      = −0.200

sum        = 0.100 + 0.080 − 0.200          = −0.020      (inside ±0.45, no clamp)
```

Read it like a conversation: P says "we're 5 ft low, push up!" I says "we've been low for a while, push up!" D says "we're already climbing fast, *ease off* or we'll overshoot!" D wins slightly, so the net push is a tiny bit *down*. The drone is coasting up nicely, and D is braking early. That's exactly what you want.

### 💻 In the code

The whole `compute()` from [`PID.hpp`](src/flight/services/PID.hpp), now fully readable:

```cpp
float error = target - actual;                                   // 4.1
float p = _kp * error;                                           // 4.2
_integral += error * dt;                                         // 4.3 running total
_integral = clamp(_integral, _outMin / _ki, _outMax / _ki);      // 4.3 anti-windup
float i = _ki * _integral;
float derivative = (error - _lastError) / dt;                    // 4.4 speedometer
float d = _kd * derivative;
_lastError = error;                                              // remember for next tick
return clamp(p + i + d, _outMin, _outMax);                       // never push past the limits
```

`computeWithError(error, dt)` is the same thing, except the caller computes the error itself. That's needed when the error isn't a simple subtraction, like yaw (Lesson 4.7) or navigation (Part 7).

### ✍️ Your turn

Roll PID gains: `Kp = 0.01, Ki = 0.001, Kd = 0.005`, clamp ±0.3. Target roll 0°, actual roll 10°, last tick's error was −10.2°. The integral so far is −2. `dt = 0.005`. Compute P, I, D, and the output.

> **Answer:**
> - error = 0 − 10 = −10
> - P = 0.01 × −10 = **−0.100**
> - integral = −2 + (−10 × 0.005) = −2.05, I = 0.001 × −2.05 = **−0.002**
> - derivative = (−10 − (−10.2)) ÷ 0.005 = 0.2 ÷ 0.005 = 40 (flip: `× 200`, so `0.2 × 200 = 40`), D = 0.005 × 40 = **+0.200**
> - sum = −0.100 − 0.002 + 0.200 = **+0.098**
>
> The drone is tilted 10° but already swinging back toward level quickly (error shrinking by 0.2° per tick). So D overrides P and brakes the swing so it doesn't overshoot past level.

**Takeaway:** *PID output = P (now) + I (history) + D (trend), clamped.*

---

## Lesson 4.6 — Feed-forward: start from a good guess

### 🧸 ELI5

Driving on the highway, you don't start each second with your foot off the gas and let the "grudge" build up to 60 mph. You **already know** roughly how much gas holds 60. You press that much, and only make small corrections. That's feed-forward: a known baseline, plus a PID that only trims around it.

### 💻 In the code

```cpp
out.baseThrottle = settings().airframe.hoverThrottle + altitudePID.compute(targetAltFt, altFt, dt);
```

The hover throttle, `"hoverThrottle": 0.50`, lives in the settings file [`flightsettings.json`](data/flightsettings.json). The altitude PID's output is clamped to ±0.45, so it's only ever a correction: throttle can range from 0.05 to 0.95 around that 0.50 baseline.

Without feed-forward, the I term would have to build the entire hover throttle up from zero. That's slow, and when the grudge finally gets big enough it overshoots. That's the "altitude hunting" the code comment mentions. (Part 8 shows *why* 0.50 is exactly the hover throttle for the sim's airframe.)

**Takeaway:** *Feed-forward supplies the known baseline. The PID only fixes what's left over.*

---

## Lesson 4.7 — Angles wrap around: the yaw error

### 🧸 ELI5

It's 11 o'clock, and you need to get to 1 o'clock. Is that 10 hours backward, or 2 hours forward? Obviously 2 forward. A clock wraps around, and so does a compass. 359° and 1° are only 2° apart.

### 🔢 Numbers

Heading 350°, target heading 10°:

```text
naive error = 10 − 350 = −340°      "turn 340° to the left" ✗ the long way round
fixed       = −340 + 360 = +20°     "turn 20° to the right"  ✓
```

The rule: **the error should always be between −180° and +180°.** If it's bigger than 180, you're going the long way round, so subtract 360. If it's less than −180, add 360.

### 💻 In the code

```cpp
float yawError = yawTargetHeading - compassHeading;
if (yawError > 180.0f) yawError -= 360.0f;
if (yawError < -180.0f) yawError += 360.0f;
float baseYawCorrection = yawPID.computeWithError(yawError, dt);
```

This is why yaw uses `computeWithError`: the error isn't a plain subtraction. It needs this fix first.

### ✍️ Your turn

Heading 20°, target 300°. What's the naive error, what's the fixed error, and which way does the drone turn?

> **Answer:** naive = 300 − 20 = 280°. That's over 180, so subtract 360: **−80°**. Turn 80° to the left (counter-clockwise), which beats 280° to the right.

**Takeaway:** *Always wrap angle errors into −180…+180, so the drone turns the short way.*

---

## Lesson 4.8 — Units detective: the yaw damping line

This lesson is a real bug that was found (and fixed) while this course was being written. It uses Lesson 0.2 (units) and Lesson 4.4 (D).

Right after the yaw PID, the code **used to** say:

```cpp
float yawCorrection = baseYawCorrection - (gyroZ * 0.02f);
```

The idea is extra damping: "if the drone is *spinning*, push against the spin." That needs a **rate**, something in **degrees per second**. For example, spinning at 20 °/s would give `20 × 0.02 = 0.4` of counter-push.

Now check what `gyroZ` actually holds. In [`Mpu6050Imu.hpp`](src/flight/hardware/Mpu6050Imu.hpp):

```cpp
out.gyroZ = filter_.getYaw();     // the fused yaw ANGLE, in degrees, not a rate
```

The project guide even warns about this: the `gyroX/Y/Z` fields hold **angles**, not raw gyro rates. So write the units under the line:

```text
yawCorrection  =  (PID output)  −  gyroZ   ×  0.02
                                   degrees    (expects deg/s)    ✗ units don't match
```

Put in numbers. If the drone happens to be facing 90°, then `90 × 0.02 = 1.8`. The motor mix clamps each motor to 0–1, so a 1.8 yaw push would drive two motors to full and two to zero. In the sim, nothing wrote `gyroZ`, so it stayed 0 and the line had no effect. That's why the sim flew fine and hid the bug.

### The fix

Give the line the rate it asked for. [`Mpu6050Imu.hpp`](src/flight/hardware/Mpu6050Imu.hpp) already had the raw gyro rate, `gz`, *before* it went into the filter. Now it's saved in its own, clearly named field:

```cpp
out.yawRateDps = -gz;    // deg/s; minus because the chip's "+" is counter-clockwise,
                         // but compass heading grows clockwise
```

and the damping line uses it, with the total capped:

```cpp
float yawCorrection = constrain(baseYawCorrection - (yawRateDps * 0.02f), -0.2f, 0.2f);
```

Now check the units: `deg/s × 0.02` gives a push, and every term matches ✓. Why the cap? Turning at 90 °/s gives `90 × 0.02 = 1.8`, which is still enough to max out two motors and shut off the other two. Capping at ±0.2 (the yaw PID's own limit) means yaw can never steal the motor range that roll and pitch need to keep the drone upright. Yaw is the least urgent of the three: a drone facing the wrong way is fine, but a drone that tips over is not.

Notice that `dps` in the name is a unit suffix (degrees per second), like `Ft` and `M` elsewhere. Naming the unit is how you stop this kind of bug from coming back.

**Takeaway:** *A unit check can find real bugs without flying anything: here, an angle is being used where a rate is expected.*

---

## Lesson 4.9 — Reading the gains

Now the gains in the `airframe` section of [`flightsettings.json`](data/flightsettings.json) are readable:

```json
"altitudePid": { "kp": 0.02,  "ki": 0.008,  "kd": 0.10,  "min": -0.45, "max": 0.45 },
"rollPid":     { "kp": 0.01,  "ki": 0.001,  "kd": 0.005, "min": -0.3,  "max": 0.3 },
"pitchPid":    { "kp": 0.01,  "ki": 0.001,  "kd": 0.005, "min": -0.3,  "max": 0.3 },
"yawPid":      { "kp": 0.005, "ki": 0.0001, "kd": 0.001, "min": -0.2,  "max": 0.2 },
"navPid":      { "kp": 0.35,  "ki": 0.0,    "kd": 0.6,   "min": -6.0,  "max": 6.0 }
```

([`MotorController.hpp`](src/flight/services/MotorController.hpp) loads them at startup; the nav gains are used twice, for forward and for right.)

The units of each gain are "output per unit of error." That makes them readable:

```text
PID        input (error)     output              Kp means...
altitude   feet              throttle trim       5 ft low → +0.10 throttle
roll       degrees           motor difference    10° off → 0.10 motor difference
nav        meters            target tilt (deg)   8 m away → lean 2.8°, max 6°
```

Notice the nav PIDs: **no I** (`Ki = 0`) and **strong D** (0.6, bigger than P). The comment explains why. Leaning too hard on long legs built up speed and made the drone overshoot and circle each waypoint. Strong D brakes on approach, and the ±6° cap keeps cruising speed modest.

### Tuning cheat sheet

```text
symptom                              usual fix
sluggish, slow to reach target       raise Kp
bounces / oscillates around target   lower Kp, or raise Kd
settles slightly off target          raise Ki (or improve feed-forward)
jittery, twitchy, noisy motors       lower Kd (D amplifies sensor noise)
huge overshoot after being stuck     integral windup: check the clamp
```

Always try a change in the sim (`run_hil.sh`) before flying.

**Takeaway:** *A gain is "push per unit of error." Read its units and you know what it does.*

---

## Part 4 checkpoint

- [ ] error = target − actual, and what its sign means
- [ ] P as a rubber band: its two weaknesses
- [ ] I as a grudge: what it fixes, what windup is, why the clamp divides by `Ki`
- [ ] D as brakes: why its sign flips automatically, and what derivative kick is
- [ ] feed-forward: why `hoverThrottle` exists
- [ ] why yaw error is wrapped into ±180
- [ ] how to read a gain's units

---

# Part 5 — Motor mixing: four wishes, four motors

## Lesson 5.1 — How a quadcopter steers with only four speeds

### 🧸 ELI5

Four people carry a table, one at each corner. To **lift** the whole table, everyone lifts harder. To **tip it forward**, the two in back lift harder and the two in front ease off. To **tip it left**, the right side lifts harder.

A quadcopter has no rudder, no flaps, nothing that moves except four propellers. Every maneuver is "some corners push harder, some push softer."

### 🖼️ Picture

From the `IMotors` plug in [`FlightIo.hpp`](src/flight/FlightIo.hpp), looking down at the drone:

```text
              FRONT
        M1 (CW)     M2 (CCW)
            ╲        ╱
             ╲      ╱
              [body]
             ╱      ╲
            ╱        ╲
        M3 (CCW)    M4 (CW)
              REAR
```

### The four wishes

The PIDs produce four "wishes." The mixer turns them into four motor speeds:

```text
wish                how it's done
base throttle       all four motors, equally          → climb or descend
pitch correction    front pair vs. rear pair          → nose up / nose down
roll correction     left pair vs. right pair          → lean left / lean right
yaw correction      one diagonal vs. the other        → spin left / spin right
```

Why diagonals for yaw? Each spinning prop pushes air, and air pushes back, twisting the drone the *opposite* way to the prop's spin (like the kick when you start a drill). M1 and M4 spin clockwise, M2 and M3 counter-clockwise (seen from above). Normally the twists cancel. Speed up one diagonal pair, slow the other, and the twists stop cancelling, so the drone rotates.

Which way? The body twists *opposite* to the props you sped up. Speed up the counter-clockwise pair (M2 + M3) and the body turns **clockwise**, so the heading goes up. That's why the mixer adds `+yaw` to M2 and M3: a positive yaw push means "turn clockwise." (A true story: an earlier version of the motor diagram had the directions backward, which would have made the yaw control push the wrong way. It was caught by exactly this reasoning, before anything was built.)

### 💻 In the code

In [`MotorController.hpp`](src/flight/services/MotorController.hpp):

```cpp
out.m1 = constrain(base + pitch + roll - yaw, 0.0f, 1.0f);   // front-left
out.m2 = constrain(base + pitch - roll + yaw, 0.0f, 1.0f);   // front-right
out.m3 = constrain(base - pitch + roll + yaw, 0.0f, 1.0f);   // rear-left
out.m4 = constrain(base - pitch - roll - yaw, 0.0f, 1.0f);   // rear-right
```

(Shortened names; the code uses `out.baseThrottle`, `out.pitchCorrection`, etc.) Lay the signs out as a table and the table-carrying picture pops out:

```text
          pitch    roll     yaw
M1  FL      +        +       −
M2  FR      +        −       +
M3  RL      −        +       +
M4  RR      −        −       −
          front+   left+    diagonals
          rear−    right−   M2/M3 vs M1/M4
```

- **+pitch** raises the front motors and lowers the rear → the nose lifts. (Firmware +pitch = nose up.)
- **+roll** raises the left motors and lowers the right → the left side lifts and the drone banks right. (Firmware +roll = banked right.)

### The clever bit: corrections don't change total lift

Add up each column: two `+` and two `−` every time. So pitch, roll, and yaw corrections **only move thrust between motors**. The total stays `4 × base`. That's why altitude control (base) and attitude control (the three corrections) don't fight each other.

### 🔢 Numbers

base 0.50, pitch +0.05, roll −0.03, yaw +0.02:

```text
M1 = 0.50 + 0.05 + (−0.03) − 0.02 = 0.50
M2 = 0.50 + 0.05 − (−0.03) + 0.02 = 0.60
M3 = 0.50 − 0.05 + (−0.03) + 0.02 = 0.44
M4 = 0.50 − 0.05 − (−0.03) − 0.02 = 0.46

total = 2.00 = 4 × 0.50   ✓ lift unchanged
```

### The catch: clamping

If `base` is already 0.95 and a motor wants +0.10, it gets clipped at 1.0. Now the corrections no longer balance, and the drone can't steer as well. That's one reason the tilt-compensation boost (Lesson 1.5) caps at 2.0, and why a drone near full throttle feels mushy.

### ✍️ Your turn

base 0.40, pitch 0, roll +0.10, yaw 0. Which motors speed up? Which way does the drone lean?

> **Answer:** M1 = 0.50, M2 = 0.30, M3 = 0.50, M4 = 0.30. The **left** motors (M1, M3) speed up, the left side rises, and the drone **banks right**. Total = 1.60 = 4 × 0.40 ✓

**Takeaway:** *Each correction adds to one half of the motors and subtracts from the other half, so the drone tilts or spins without changing total lift.*

---

# Part 6 — GPS: turning latitude/longitude into meters

## Lesson 6.1 — Latitude and longitude are angles, not distances

### 🧸 ELI5

Picture the Earth as a ball with a pin through the middle. **Latitude** is how far up or down from the equator you are, measured as an **angle** from the center of the Earth. **Longitude** is how far around (east/west) you are, also an angle.

So a GPS reading like `(35.9812°, −78.8931°)` isn't "meters from somewhere." It's two angles. The PIDs want meters ("12 m north, 5 m east"). This part is about converting.

### 🔢 Numbers: latitude is easy

Going north one degree of latitude is always about the same distance, because every north-south line is a full-size circle around the Earth:

```text
Earth's circumference ≈ 40,075 km
1° of latitude        ≈ 40,075 km ÷ 360 ≈ 111,320 m     (the number in the code)

0.001°   of latitude ≈ 111 m
0.0001°  of latitude ≈  11 m
0.00001° of latitude ≈ 1.1 m
```

So GPS coordinates need **five or six decimal places** just to get to meter-level precision.

### A side note on `double`

A `float` holds only about 7 significant digits. Near 36°, the smallest step a float can take is about 0.000004°, which is roughly **40 cm**. Near longitude −79° it's about 70 cm. Every calculation rounds to those steps, so errors pile up. That's why every lat/lon in the code is a **`double`** (about 16 digits), and only the small differences in meters are converted to `float`.

**Takeaway:** *Lat/lon are angles. 1° of latitude ≈ 111,320 m.*

---

## Lesson 6.2 — Longitude shrinks as you go north

### 🧸 ELI5

Peel an orange. The segments are fat at the middle and pinch to nothing at the top. Longitude lines are the same: they're far apart at the equator and meet at the poles. So one degree of longitude is a **shorter** distance the farther you are from the equator.

### 🖼️ Picture

```text
                  north pole (lines meet)
                      ·
                   ╱  │  ╲
                 ╱    │    ╲      ← near the pole: 1° of longitude is tiny
               ╱      │      ╲
             ╱        │        ╲
           ──────────────────────  ← equator: 1° of longitude ≈ 111 km
```

How much shorter? The circle of latitude you're standing on has radius `R × cos(latitude)`. At the equator, `cos(0°) = 1`, full size. At the pole, `cos(90°) = 0`, zero size. It's the Lesson 1.1 shadow again: the "along the ground" shadow of the Earth's radius.

```text
meters per degree of longitude = 111,320 × cos(latitude)
```

### 🔢 Numbers

```text
latitude     cos       meters per 1° longitude
  0°        1.000        111,320
 36°        0.809         90,080     (roughly North Carolina)
 60°        0.500         55,660
 90°        0.000              0
```

### 💻 In the code

In `lineFollowNorthEast()`, [`NavMath.hpp`](src/flight/services/NavMath.hpp):

```cpp
float mPerLat = 111320.0f;
float mPerLon = 111320.0f * cos(radians(prevLat));
float posE = (curLon - prevLon) * mPerLon;   // degrees of longitude → meters east
float posN = (curLat - prevLat) * mPerLat;   // degrees of latitude  → meters north
```

That's the entire conversion. Over a small area, like a field, the Earth is flat enough that this is accurate to well under a meter. (This flat-patch trick has a name: the **equirectangular approximation**.)

### ✍️ Your turn

At latitude 36°, you move from longitude −78.89300° to −78.89250°, and latitude doesn't change. How many meters east did you go?

> **Answer:** Δlon = −78.89250 − (−78.89300) = +0.00050°. Meters = 0.00050 × 90,080 ≈ **45 m east**. (Positive, because moving toward a less-negative longitude is moving east.)

**Takeaway:** *Meters east = Δlongitude × 111,320 × cos(latitude). Meters north = Δlatitude × 111,320.*

---

## Lesson 6.3 — Haversine: distance on a ball

### 🧸 ELI5

For long distances, the flat-patch trick breaks: the Earth curves away underneath. The **haversine formula** gives the true distance along the curved surface, like stretching a string tightly across a globe between two cities.

### 💻 In the code

`gpsDistanceMeters()` in [`NavMath.hpp`](src/flight/services/NavMath.hpp):

```cpp
float a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
return R * 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
```

**Don't memorize this.** Read it at the level of "what's each part doing":

```text
a                   a number from 0 to 1 that grows with how far apart the points are.
                    Notice the cos(lat) factor: the "longitude shrinks" idea from 6.2.

atan2(√a, √(1−a))   the pieces → angle move from Lesson 1.8. √a is the sticks-out
                    piece and √(1−a) is the along piece of a length-1 arrow
                    (check: (√a)² + (√(1−a))² = 1). It returns HALF the angle
                    between the two points, as seen from the center of the Earth.

2 × …               doubles it to the full angle, in radians.

R × angle           arc length = radius × angle (in radians). This is exactly the
                    definition of a radian from Lesson 1.3. R = 6,371,000 m.
```

So even this intimidating formula is built from things you already know: sines, cosines, `atan2`, and "radius × radians = distance along the edge."

The mission uses it to decide when a waypoint has been reached:

```cpp
float distM = gpsDistanceMeters(gps.lat, gps.lon, wp.lat, wp.lon);
if (distM < settings().flight.waypointAcceptRadiusM) { /* advance to the next waypoint */ }
```

`"waypointAcceptRadiusM"` is 4 m in [`flightsettings.json`](data/flightsettings.json).

**Takeaway:** *Haversine = true distance on a sphere. It ends with `R × angle`, the radian definition of arc length.*

---

## Lesson 6.4 — Distance + bearing ⇄ north + east

This is Lesson 1.2's rule with a compass. A bearing is measured **from north**, so cosine is the **north** piece:

```cpp
// NavMath.hpp
inline void bearingToNorthEast(float distM, float bearingDeg, float& northM, float& eastM) {
  float rad = radians(bearingDeg);
  northM = distM * cos(rad);
  eastM  = distM * sin(rad);
}
```

### 🔢 Numbers

100 m at bearing 60°:

```text
north = 100 × cos(60°) = 100 × 0.500 = 50.0 m
east  = 100 × sin(60°) = 100 × 0.866 = 86.6 m
check: √(50² + 86.6²) = √(2500 + 7500) = 100 ✓
```

And back again, with `atan2(east, north)` from Lesson 1.8:

```text
atan2(86.6, 50) = 60°  ✓
```

### ✍️ Your turn

200 m at bearing 180°. North and east?

> **Answer:** north = 200 × cos(180°) = 200 × −1 = **−200 m** (that's south). east = 200 × sin(180°) = **0**. Bearing 180° is due south ✓.

**Takeaway:** *From north: `north = d·cos(bearing)`, `east = d·sin(bearing)`. `atan2(east, north)` undoes it.*

---

# Part 7 — Line following: the carrot

## Lesson 7.1 — Why not just fly straight at the waypoint?

### 🧸 ELI5

When you ride a bike, you don't stare at your front wheel. You look a little way **down the road** and steer toward that. If you drift off, the point you're looking at is still on the road, so you naturally curve back onto it.

Now imagine flying a square fence line. If the drone aims straight at the next corner from wherever it currently is, any drift or wind makes it cut across at an angle instead of following the edge. The fix: dangle a **carrot** on the line, a bit ahead of the drone, and always chase the carrot. The drone gets pulled back onto the line and follows it corner to corner.

### 🖼️ Picture

```text
  previous waypoint                                    next waypoint
        ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●━━━━━━━━━━━━━━━━━━━━●
                            ┆         🥕 carrot
                  shadow ─► ┊       ╱  (18 m ahead of the shadow,
                  on line   ┊     ╱     ON the line)
                            ┊   ╱
                            ┊ ╱  ← steer this way:
                            ● drone      back toward the line AND forward
```

Four steps, each one a Part 2 lesson:

1. Describe the leg as a **unit vector** (Lesson 2.4).
2. Find the drone's **shadow on the line**: how far along the leg it is (projection, Lesson 2.6).
3. Put the carrot **18 m further along** the line (scaling, Lesson 2.4).
4. The steering arrow is **carrot − me** (subtraction, Lesson 2.3).

**Takeaway:** *Chase a point on the line, a fixed distance ahead of your shadow, and you'll converge onto the line.*

---

## Lesson 7.2 — Walking through `lineFollowNorthEast()` with numbers

The function in [`NavMath.hpp`](src/flight/services/NavMath.hpp), with a worked example alongside.

**Setup.** Put the origin at the previous waypoint. The leg goes to a waypoint **60 m east, 80 m north**. The drone is at **40 m east, 20 m north**, a bit off to the side. The lookahead is `"waypointLookaheadM": 18` m (in the settings file).

### Step 0 — degrees to meters (Lesson 6.2)

```cpp
float segE = (wpLon  - prevLon) * mPerLon;   // 60   the leg, as a vector
float segN = (wpLat  - prevLat) * mPerLat;   // 80
float posE = (curLon - prevLon) * mPerLon;   // 40   the drone, from the same origin
float posN = (curLat - prevLat) * mPerLat;   // 20
```

### Step 1 — the leg's length and direction (Lessons 2.2, 2.4)

```cpp
float segLen = sqrt(segE * segE + segN * segN);   // √(3600 + 6400) = 100 m
if (segLen < 1.0f) { ... aim straight at the waypoint ... }   // don't divide by ~0
float uE = segE / segLen, uN = segN / segLen;     // (0.6, 0.8)  unit vector along the leg
```

### Step 2 — the drone's shadow on the line (Lesson 2.6)

```cpp
float proj = posE * uE + posN * uN;   // 40×0.6 + 20×0.8 = 24 + 16 = 40 m along the leg
if (proj < 0.0f) proj = 0.0f;         // behind the start? treat it as the start
```

### Step 3 — place the carrot (Lesson 2.4)

```cpp
float carrot = proj + lookaheadM;                      // 40 + 18 = 58 m along
if (carrot > segLen) carrot = segLen;                  // never past the waypoint
float carrotE = uE * carrot, carrotN = uN * carrot;    // 58 × (0.6, 0.8) = (34.8, 46.4)
```

### Step 4 — the arrow from me to the carrot (Lesson 2.3)

```cpp
eastM  = carrotE - posE;     // 34.8 − 40 = −5.2 m    (a little WEST: back toward the line)
northM = carrotN - posN;     // 46.4 − 20 = 26.4 m    (mostly NORTH: forward along the leg)
```

Read the answer: "go 26.4 m north and 5.2 m west." The north part carries the drone forward along the leg. The small west part pulls it back onto the line. That's the bike-riding picture, in numbers.

### Step 5 — turn meters into lean (Part 4)

Back in [`MissionPhase.hpp`](src/flight/phases/MissionPhase.hpp):

```cpp
float forwardM, rightM;   // world north/east -> the drone's own forward/right
northEastToForwardRight(northM, eastM, shared.raw.compassHeadingDeg, forwardM, rightM);
shared.cruise_mission.targetRollDeg  = motorController_->rightNavigationCorrection(rightM, navDt);
shared.cruise_mission.targetPitchDeg = motorController_->forwardNavigationCorrection(forwardM, navDt);
```

First, north/east is turned into the drone's own forward/right. That's Lesson 10.1; for now, picture the nose pointing north, so forward = north (26.4) and right = east (−5.2). Then the nav PIDs (`Kp = 0.35` degrees per meter, clamp ±6°) turn those meters into lean angles. Just the P part:

```text
right:   0.35 × −5.2 = −1.8°   (lean gently to fix the sideways drift)
forward: 0.35 × 26.4 =  9.2°   → clamped to 6°  (cruise at the max lean)
```

Those lean angles become the *targets* for the roll and pitch PIDs (Part 4), which drive the motor mix (Part 5). Every Part so far just ran, in order.

(Which physical direction a positive pitch or roll moves the drone depends on the frame conventions. Those were locked down and tested in the sim; see §8 of the project guide.)

### ✍️ Your turn

Same leg (unit vector (0.6, 0.8), length 100). The drone is at (70, 90), past the end of the leg. Where's the carrot, and what's the steering arrow?

> **Answer:** proj = 70×0.6 + 90×0.8 = 42 + 72 = 114. carrot = 114 + 18 = 132, which is past 100, so it's clamped to **100**: the waypoint itself, (60, 80). Arrow = (60 − 70, 80 − 90) = **(−10, −10)**: back toward the waypoint. The clamp stops the carrot from running off past the corner.

**Takeaway:** *Line following = unit vector → projection → carrot ahead → carrot minus me → nav PID → lean.*

---

# Part 8 — Forces: why it hovers, why it falls

This part mostly reads [`QuadSim.h`](lib/QuadSim/src/QuadSim.h), the physics that the on-chip sim runs. It's the "④ PHYSICS" box from Lesson 0.1. The firmware doesn't run this math on a real drone (the real world does it for free), but you need it to understand *why* the controller does what it does.

## Lesson 8.1 — F = ma: push, mass, and speeding up

### 🧸 ELI5

Push an empty shopping cart and it zooms off. Push a full one just as hard and it barely creeps. Push harder, and either one speeds up faster.

```text
acceleration = force ÷ mass          (usually written F = m·a)
```

**Acceleration** means "how fast the speed is changing," the derivative of velocity from Part 3. Units: m/s² ("meters per second, every second").

The key subtlety: **only the *leftover* force accelerates you.** If two forces cancel, the acceleration is zero, even if both forces are huge.

### 🖼️ Picture — a free-body diagram

Draw every force on the drone as an arrow:

```text
                ▲  thrust (from the props)
                │
            [ drone ]
                │
                ▼  weight = m × g  (gravity)
                ▼  drag (air resistance, always against the motion)
```

`g = 9.81 m/s²`, the acceleration of gravity. Weight is `m × g`. Add up the arrows (up is +, down is −). The leftover is what accelerates the drone.

### 🔢 Numbers

The sim's drone: `mass = 1.2 kg`.

```text
weight = 1.2 × 9.81 = 11.77 N (pointing down)

thrust 11.77 N →  leftover   0     → a = 0          hovers (or keeps drifting at steady speed)
thrust 15.00 N →  leftover  +3.23  → a = +2.69 m/s² speeds up upward
thrust  8.00 N →  leftover  −3.77  → a = −3.14 m/s² speeds up downward
```

**Zero acceleration doesn't mean stopped.** It means "speed isn't changing." A drone climbing at a steady 2 m/s has thrust exactly equal to weight (ignoring drag). Thrust only has to exceed weight while it's *speeding up* the climb.

### 💻 In the code

```cpp
float az = (tw[2] - p_.dragZ * s_.vel[2]) / p_.mass - kGravity;
```

Read it as a free-body diagram:

```text
tw[2]                    thrust's upward piece (after tilting, see Part 10)
− dragZ × vel[2]         drag: pushes against the vertical speed
÷ mass                   F = ma → a = F ÷ m
− kGravity               gravity's acceleration, 9.81, always down
```

Drag is written as `drag coefficient × speed`: the faster you go, the harder the air pushes back. That's why a falling drone eventually stops speeding up (terminal velocity): drag grows until it matches weight.

**Takeaway:** *Acceleration = leftover force ÷ mass. Balanced forces mean steady speed, not necessarily stopped.*

---

## Lesson 8.2 — Thrust grows with speed *squared* (and why hover is 0.50)

### 🧸 ELI5

Stick your hand out a car window. At 30 mph you feel a push. At 60 mph it's not twice as strong, it's about **four times** as strong. Air force grows with the *square* of speed. Propellers work the same way.

### 🔢 Numbers

From `QuadSim.h`:

```cpp
float w = c * p_.maxRotorSpeed;   // throttle (0..1) → rotor speed
T[i] = p_.kThrust * w * w;        // thrust = k × speed²
```

Throttle sets speed linearly, but thrust goes as speed squared. So **thrust goes as throttle squared**:

```text
throttle   thrust (as a fraction of max)
 0.25          0.0625      (1/16)
 0.50          0.25        (1/4)
 0.75          0.5625
 1.00          1.00
```

Half throttle gives only a quarter of the max thrust.

The on-chip sim's airframe ([`SimWorld.hpp`](src/apps/sim/SimWorld.hpp)) is set up so that full throttle gives **4× the drone's weight** (a "4:1 thrust-to-weight ratio"):

```text
max thrust  = 4 × 11.77 = 47.09 N
hover: thrust = weight
       47.09 × throttle² = 11.77
       throttle²         = 0.25
       throttle          = 0.50
```

**That's where `"hoverThrottle": 0.50` comes from.** (Lesson 4.6.) A real airframe with a different thrust-to-weight ratio hovers at a different throttle, which is why the project guide says to tune it per airframe. In general: `hover throttle = √(1 ÷ thrust-to-weight)`.

### ✍️ Your turn

Sim drone at throttle 0.75. What's the thrust, the leftover force, and the upward acceleration? (Ignore drag.)

> **Answer:** thrust = 47.09 × 0.75² = 47.09 × 0.5625 = **26.49 N**. Leftover = 26.49 − 11.77 = **14.72 N**. a = 14.72 ÷ 1.2 = **12.3 m/s²** upward (flip: `÷ 1.2 = × 1/1.2 ≈ × 0.833`, so `14.72 × 0.833 ≈ 12.3`), more than gravity's 9.81. That's a strong climb.

**Takeaway:** *Thrust ∝ throttle². With 4:1 thrust-to-weight, hover is exactly half throttle.*

---

## Lesson 8.3 — Integration: from acceleration to position

### 🧸 ELI5

You're walking with your eyes closed, but someone tells you your speed every second. To know where you are, you keep a running tally: "I was here, I walked at 1 m/s for a second, so now I'm 1 m further." That's integration (Lesson 3.2), done in tiny steps. It's also called **Euler integration**.

### 🔢 Numbers

The two lines from Lesson 0.2:

```text
velocity += acceleration × dt
position += velocity     × dt
```

A drone at 10 m altitude with its motors cut (a = −9.81 m/s²), stepped at `dt = 0.1` s:

```text
tick   velocity (m/s)                 position (m)
 0       0                              10.000
 1       0 − 9.81×0.1   = −0.981        10 − 0.981×0.1 = 9.902
 2    −0.981 − 0.981    = −1.962        9.902 − 0.196  = 9.706
 3    −1.962 − 0.981    = −2.943        9.706 − 0.294  = 9.411
```

The exact answer after 0.3 s (from the Calc 1 formula `½·g·t²`) is 9.559 m. Euler said 9.411. It's off by about 15 cm because it assumes the speed stays constant during each 0.1 s slice.

**Smaller steps fix it.** With `dt = 0.01` (30 tiny steps), Euler gives 9.544 m, which is off by only 1.5 cm. Ten times smaller steps, ten times smaller error.

### 💻 In the code

That's exactly why QuadSim sub-steps:

```cpp
constexpr float kSubStep = 0.001f;     // internal integration step (s)
...
int n = (int)std::ceil(dt / kSubStep); // a 5 ms physics tick = 5 tiny 1 ms steps
for (int i = 0; i < n; ++i) integrate(motor, h);
```

and inside `integrate()`:

```cpp
s_.vel[2] += az * h;           // velocity += acceleration × dt
s_.pos[2] += s_.vel[2] * h;    // position += velocity × dt
```

**Takeaway:** *Simulation = add up `rate × dt` every tick. Smaller `dt` means a more accurate answer.*

---

# Part 9 — Rotation: torque and spin

## Lesson 9.1 — Rotation is a mirror image of straight-line motion

### 🧸 ELI5

Everything in Part 8 has a spinning twin. Once you see the pairs, rotation is just F = ma wearing a costume:

```text
straight line                    spinning
position (m)                     angle (rad)
velocity (m/s)                   angular velocity, "spin rate" (rad/s)
acceleration (m/s²)              angular acceleration (rad/s²)
force (N)                        torque (N·m)
mass (kg)                        moment of inertia (kg·m²)
a = F ÷ m                        α = τ ÷ I
```

(α "alpha" is angular acceleration, τ "tau" is torque. Just names.)

**Takeaway:** *Rotation has the same shape as F = ma: angular acceleration = torque ÷ moment of inertia.*

---

## Lesson 9.2 — Torque: force × lever arm

### 🧸 ELI5

Push a door near the hinge: it barely moves. Push at the handle, far from the hinge: it swings easily. Same push, different distance from the pivot. **Torque** is the turning power:

```text
torque = force × distance from the pivot     (the "lever arm")
```

### 🖼️ Picture

For roll, the pivot is the drone's front-to-back centerline. The left motors push up on one side, the right motors on the other:

```text
        left motors                      right motors
        push T_left                      push T_right
            ▲                                ▲
            │                                │
            ●━━━━━━━━━━━━━━━━●━━━━━━━━━━━━━━━●
            │◄────── d ─────►│◄────── d ────►│
                        centerline (pivot)

net roll torque = d × (T_left − T_right)
```

### 💻 In the code

In `QuadSim.h`:

```cpp
const float d = p_.armLength * 0.70710678f;          // lever arm on each axis
float tauX = d * ((T[0] + T[2]) - (T[1] + T[3]));    // roll:  left (FL+RL) − right (FR+RR)
float tauY = d * ((T[2] + T[3]) - (T[0] + T[1]));    // pitch: rear (RL+RR) − front (FL+FR)
```

Why `armLength × 0.707`? In an X layout, each motor sits on a diagonal arm at 45° to the centerline. Its sideways distance from the roll axis is the "sticks out" shadow of the arm: `arm × sin(45°) = arm × 0.707`. Lesson 1.1 again. With a 0.23 m arm, `d = 0.163 m`.

**Takeaway:** *Torque = force × lever arm. The drone rolls when one side pushes harder than the other.*

---

## Lesson 9.3 — Moment of inertia: how hard it is to spin

### 🧸 ELI5

A figure skater spins slowly with arms out, then pulls them in and spins fast. Same skater, same mass, but mass **far from the center** is harder to spin. That's **moment of inertia**, `I`: rotation's version of mass, where *where* the mass sits matters as much as how much there is.

### 🔢 Numbers

Sim drone: `Ixx = 0.015 kg·m²` (roll). The left motors push 3.2 N each, the right 2.8 N each:

```text
τ = 0.163 × ((3.2 + 3.2) − (2.8 + 2.8)) = 0.163 × 0.8 = 0.130 N·m
α = τ ÷ I = 0.130 ÷ 0.015 = 8.7 rad/s²
```

(Flip the `÷ 0.015`: it's `× 1/0.015 ≈ × 66.7`, so `0.130 × 66.7 ≈ 8.7`. Dividing by a small `I` multiplies by a big number — little drones spin up fast.)

After just 0.1 s at that rate: spin rate = 8.7 × 0.1 = 0.87 rad/s, about **50°/s**. A tiny 0.4 N difference per motor flips the drone's attitude quickly. That's why the roll and pitch loops run 200 times a second, not 10.

### 💻 In the code

```cpp
float wdotX = (tauX - gyroX) / Ix;     // α = τ ÷ I   (gyroX here is a 3D spin correction; see below)
s_.omega[0] += wdotX * h;              // spin rate += α × dt   (integration again, Lesson 8.3)
```

The `gyroX/Y/Z` terms inside QuadSim (not the IMU fields!) are an extra effect that only shows up when something spins on two axes at once. They're small at normal flying speeds. Treat them as "a 3D correction" for now.

**Takeaway:** *Angular acceleration = torque ÷ inertia. Small thrust differences produce fast rotation, so attitude control must be fast.*

---

## Lesson 9.4 — Yaw: spinning with no lever arm

Yaw doesn't come from thrust differences. It comes from the **reaction twist** of the propellers (Lesson 5.1). Each prop drags on the air, so the air twists the motor the other way, and that twist grows with speed squared, just like thrust:

```cpp
float tauZ = p_.kTorque * (wsq[0] - wsq[1] - wsq[2] + wsq[3]);   // spin dirs [+,-,-,+]
```

The signs follow the spin directions. The FL/RR pair twists one way, the FR/RL pair the other. When all four match, they cancel. That's exactly the diagonal pattern in the motor mix.

**Takeaway:** *Yaw comes from propeller drag twisting the body. The two diagonals twist opposite ways, so the mix speeds one diagonal and slows the other.*

---

# Part 10 — Orientation: frames, rotating arrows, and quaternions

## Lesson 10.1 — Two points of view: world frame and body frame

### 🧸 ELI5

Give a friend directions over the phone. "Go **north**" works no matter which way they're facing. That's the **world frame**: north, east, up, fixed to the Earth. "Go **forward**, then **right**" depends entirely on which way they're facing. That's the **body frame**: forward, right, up, fixed to the drone.

GPS thinks in world frame ("the carrot is 26 m north"). Motors think in body frame ("pitch the nose down to go forward"). Something has to translate.

### 🖼️ Picture

The drone faces heading `h` (measured clockwise from north). Its "forward" direction, written in world terms, is a unit arrow at bearing `h`. Its "right" is a unit arrow at bearing `h + 90°`:

```text
                north
                  ▲
                  │   forward (heading h)
                  │  ╱
                  │ ╱ h
                  │╱
                  ●─────► east
                   ╲
                    ╲  right (heading h + 90°)
```

### 🔢 Numbers — translating is just two dot products

"How much of the error points **forward**?" is the projection question from Lesson 2.6: dot the error with the forward unit vector.

```text
forward unit vector, in (north, east):  (cos h,  sin h)       bearing h
right   unit vector, in (north, east):  (−sin h, cos h)       bearing h + 90°

forward error =  north·cos(h) + east·sin(h)
right error   = −north·sin(h) + east·cos(h)
```

Drone facing **east** (h = 90°), target **10 m north**:

```text
forward = 10·cos(90°) + 0·sin(90°) =   0
right   = −10·sin(90°) + 0·cos(90°) = −10       "10 m to my LEFT"  ✓
```

Facing east, north is on your left. ✓

Drone facing 30°, target 10 m north:

```text
forward = 10 × 0.866 = 8.66     mostly ahead
right   = −10 × 0.5  = −5.00    a bit to the left  ✓
```

These two lines are called a **rotation** (a 2D rotation matrix, if you remember linear algebra).

### 💻 In the code

`northEastToForwardRight()` in [`NavMath.hpp`](src/flight/services/NavMath.hpp) is exactly those two lines:

```cpp
float h = radians(headingDeg);
forwardM =  northM * cos(h) + eastM * sin(h);
rightM   = -northM * sin(h) + eastM * cos(h);
```

Every phase that steers by GPS calls it before its nav PIDs: forward → pitch, right → roll.

**A true story.** This function didn't exist when this course was first written. The code used to send north straight to pitch and east straight to roll. That's only right when the nose points north (h = 0°). The sim hid it: it held the nose at north but told the compass "facing the waypoint," so everything looked fine. On a real drone the nose turns toward each waypoint, and a laptop test showed the old code getting lost after the second corner of a square route. Now the sim really turns its nose and reports the true heading, so the sim checks this math every flight.

### ✍️ Your turn

Drone facing south (h = 180°). The carrot is 20 m north. What are forward and right?

> **Answer:** forward = 20·cos(180°) = **−20** (20 m *behind*). right = −20·sin(180°) = **0**. Facing south, north is directly behind you ✓.

**Takeaway:** *World frame is fixed to the Earth; body frame turns with the drone. Converting between them is two dot products with cos/sin of the heading.*

---

## Lesson 10.2 — Why three angles (roll, pitch, yaw) cause trouble

### 🧸 ELI5

Roll, pitch, and yaw are three dials, turned **in a specific order**. That's great for humans to read, but it has two problems:

1. **Order matters.** Pitch up 90° then yaw 90° is a different pose from yaw 90° then pitch up 90°. Try it with a book on your desk.
2. **Gimbal lock.** Point the nose straight up (pitch 90°). Now "roll" (spinning around the nose) and "yaw" (spinning around the vertical) are *the same motion*: the nose axis *is* the vertical. You've lost a dial. The math divides by `cos(90°) = 0` and blows up.

A drone that flips or tumbles can hit that pose, and then the math breaks exactly when you need it most.

**Takeaway:** *Three angles are easy to read but depend on order and break at pitch ±90°.*

---

## Lesson 10.3 — Quaternions: "spin this much around this axis"

### 🧸 ELI5

Every change of orientation, no matter how complicated, can be done as **one single twist around one single axis**. Stick a skewer through the drone at the right angle, twist it by the right amount, and you've got it. That's Euler's rotation theorem.

So describe orientation as:

```text
axis   = which way the skewer points   (a unit vector, 3 numbers)
angle  = how far to twist             (1 number)
```

A **quaternion** is that axis-and-angle, packed into four numbers `(w, x, y, z)` in a slightly odd way:

```text
w         = cos(angle ÷ 2)
(x, y, z) = axis × sin(angle ÷ 2)
```

Why half the angle? It's what makes the multiplication rules work out. You can take that on faith. What matters is the result: **no gimbal lock, no order problems, and no division that can blow up.**

### 🔢 Numbers

```text
no rotation at all:   angle 0°  → w = cos 0° = 1, xyz = axis × 0   → (1, 0, 0, 0)
yaw 90° (axis = up):  angle 90° → w = cos 45° = 0.707
                                   xyz = (0, 0, 1) × sin 45°        → (0.707, 0, 0, 0.707)
roll 180° (axis = x): angle 180° → w = cos 90° = 0
                                   xyz = (1, 0, 0) × sin 90°        → (0, 1, 0, 0)
```

Check any of them: `w² + x² + y² + z² = 1`. A rotation quaternion is always a **unit** quaternion, length 1, like a unit vector. (That's the `cos² + sin² = 1` fact from Lesson 1.1 again.)

### 💻 In the code

In `QuadSim.h`, "level, no rotation" is exactly that first row:

```cpp
s.quat[0] = 1.0f; s.quat[1] = s.quat[2] = s.quat[3] = 0.0f;   // (w,x,y,z) = (1,0,0,0)
```

**Takeaway:** *A quaternion is "twist by this angle around this axis," packed as `(cos(½θ), axis·sin(½θ))`. Always length 1.*

---

## Lesson 10.4 — Updating a quaternion each tick

### 🧸 ELI5

Same pattern as Lesson 8.3: "new = old + rate × dt." The gyro says how fast the drone is spinning. Nudge the quaternion by that much. Then, because tiny rounding errors creep in, **re-normalize** it back to length 1 (Lesson 2.4).

### 💻 In the code

```cpp
// integrate quaternion: qdot = 0.5 * q (x) (0, omega)
float dw = 0.5f * (-x0 * ox - y0 * oy - z0 * oz);
float dx = 0.5f * ( w0 * ox + y0 * oz - z0 * oy);
...
w0 += dw * h; x0 += dx * h; ...                      // new = old + rate × dt
float nrm = std::sqrt(w0*w0 + x0*x0 + y0*y0 + z0*z0);
s_.quat[0] = w0 / nrm; ...                           // divide by length → back to length 1
```

You don't need to derive the `dw, dx, …` lines. They're the "rate of change of a quaternion, given a spin rate" formula. Recognize the shape: **compute a rate → multiply by dt → add → normalize.**

**Takeaway:** *Quaternion update = old + rate × dt, then normalize to length 1.*

---

## Lesson 10.5 — Using the quaternion: tilting the thrust, and reading angles back

### Tilting thrust into the world

The props push along the drone's own "up." Physics (Part 8) needs that push in world terms (north/east/up). So QuadSim rotates the body-frame thrust arrow `(0, 0, T)` into the world:

```cpp
float tw[3]; rotateBodyToWorld(0.0f, 0.0f, thrust, tw);
```

For a drone tilted by `θ`, `tw` comes out with an up piece of `T·cos(θ)` and a sideways piece of `T·sin(θ)`. That's Lesson 1.4, computed by a quaternion instead of by hand.

### Reading roll/pitch/yaw back out

Humans and the firmware's PIDs want degrees. `rpyDeg()` converts the quaternion back to three angles, and look what it uses:

```cpp
roll  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
pitch = std::asin(sp);
yaw   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
```

**`atan2`**, the same pieces → angle tool from Lesson 1.8. Each line builds a "sticks out" piece and an "along" piece from the quaternion, and `atan2` turns them into an angle without losing the quadrant.

### On the real drone: Madgwick

On real hardware, [`Mpu6050Imu.hpp`](src/flight/hardware/Mpu6050Imu.hpp) uses a **Madgwick filter**, which keeps its own quaternion internally. 🧸 It combines two imperfect senses:

- the **gyro** (how fast am I spinning?) is smooth and fast, but integrating it slowly **drifts**, like walking with your eyes closed;
- the **accelerometer** (which way is gravity?) never drifts, but it's **noisy** and gets fooled by vibration.

Madgwick mostly trusts the gyro, and keeps gently nudging it toward the accelerometer's idea of "down." You get smooth *and* drift-free. Then it outputs roll/pitch/yaw, which is what the PIDs read.

**Takeaway:** *The quaternion tilts the thrust into the world, and `atan2` turns it back into roll/pitch/yaw for the PIDs.*

---

# Part 11 — One whole tick, every lesson at once

Here's the payoff. Follow one moment of a MISSION flight through the real code, and name the lesson behind every step.

### The slow loop: `navTick()`, 10 times a second ([`MissionPhase.hpp`](src/flight/phases/MissionPhase.hpp))

```text
 1. Read GPS: lat/lon as doubles ..................................... 6.1  angles, not meters
 2. gpsDistanceMeters(): how far to the waypoint ..................... 6.3  haversine, R × angle
 3. Close enough (< 4 m)? advance to the next waypoint
 4. gpsBearing(): which way it is .................................... 1.8  atan2(east, north), wrap 0–360
    → becomes yawTargetHeading
 5. lineFollowNorthEast(): lat/lon → meters .......................... 6.2  × 111,320 × cos(lat)
      leg length, unit vector ........................................ 2.2, 2.4
      how far along the leg (projection) ............................. 2.6  dot product
      carrot 18 m ahead, carrot − me ................................. 2.3
 6. nav PIDs: meters → target lean angles (±6°) ...................... 4.9  P + strong D, no I
    north/east → forward/right using the compass heading ............. 10.1
```

### The fast loop: `physicsTick()`, 200 times a second ([`MotorController.hpp`](src/flight/services/MotorController.hpp))

```text
 7. altitude PID + hover feed-forward → base throttle ................ 4.5, 4.6
 8. tilt compensation: × 1/(cos roll · cos pitch), cap 2.0 ........... 1.3, 1.5  radians, cosine
 9. roll PID, pitch PID: target lean vs. actual lean ................. 4.5
10. yaw error wrapped into ±180, yaw PID, rate damping, cap ±0.2 ..... 4.7, 4.8
11. mix: base ± pitch ± roll ± yaw → M1..M4, clamp 0..1 .............. 5.1
12. Motors::writeMix() → the four ESCs
```

### The world answers (in the sim: [`QuadSim.h`](lib/QuadSim/src/QuadSim.h))

```text
13. throttle → rotor speed → thrust = k × speed² ..................... 8.2
14. torque from thrust differences (roll, pitch) and prop drag (yaw) . 9.2, 9.4
15. angular acceleration = τ ÷ I, integrate to spin rate ............. 9.3
16. quaternion += rate × dt, normalize ............................... 10.4
17. rotate thrust into the world, F = ma with gravity and drag ....... 10.5, 8.1
18. velocity += a·dt, position += v·dt (1 ms sub-steps) .............. 8.3
19. quaternion → roll/pitch/yaw via atan2 → fake sensors ............. 10.5
        │
        └────► back to step 1
```

That's the broomstick balancing loop from Lesson 0.1, spelled out in 19 steps. Every step is something you've now worked with real numbers.

**If you can narrate this list from memory, explaining *why* each step does what it does, you've mastered the course.**

---

# Part 12 — Practice, reference, and study routine

## Practice sets (answers at the end of each set)

### Set A — Trig

1. A drone makes 12 N of thrust, tilted 30° from vertical. Up piece? Sideways piece?
2. What's `tiltFactor` at roll 30°, pitch 30°? Does the cap kick in?
3. Convert 135° to radians.
4. Waypoint: 8 m east, 6 m south. What's `atan2(east, north)`? What bearing does `gpsBearing()` report after the wrap?
5. Why is `cos(0°) = 1` the right answer for a level drone's up piece?

> **Answers:**
> 1. up = 12 × 0.866 = **10.39 N**, sideways = 12 × 0.5 = **6 N**.
> 2. 1 / (0.866 × 0.866) = 1 / 0.75 = **1.33**. No cap: it's between 1 and 2.
> 3. 135 × π/180 = **2.356 rad** (that's ¾π).
> 4. north = −6. atan2(8, −6) ≈ **126.9°**. Already positive: +360 = 486.9, fmod → **126.9°**, between east (90°) and south (180°) ✓.
> 5. Level means the thrust points exactly along "up," the line we measure from. All of it is the along piece, and none sticks out.

### Set B — Vectors

1. Me (5, 5), target (−10, 25). Arrow from me to target? Distance?
2. Normalize (−8, 6).
3. `(3, 4) · (4, −3)` = ? What does it mean?
4. Road direction (0.8, 0.6). You're at (10, 20). How far along the road are you?

> **Answers:**
> 1. (−15, 20). √(225 + 400) = **25 m**.
> 2. Length √(64 + 36) = 10 → **(−0.8, 0.6)**.
> 3. 12 − 12 = **0**: perpendicular.
> 4. 10×0.8 + 20×0.6 = 8 + 12 = **20 m**.

### Set C — Calculus and PID

1. Altitude readings 10.0, 10.3 ft, 0.1 s apart. Rate of climb?
2. Altitude PID (Kp 0.02, Ki 0.008, Kd 0.10). Error 2 ft, last error 2 ft, integral 0. dt = 0.005. P, I, D, output?
3. The drone keeps hovering 1 ft below target forever. Which term is too weak, and why?
4. Heading 170°, target −170° (i.e., 190°). Wrapped yaw error?
5. What's the max possible I push for the nav PIDs, and why?

> **Answers:**
> 1. (10.3 − 10.0) ÷ 0.1 = **3 ft/s** up (flip: `÷ 0.1 = × 10`, so `0.3 × 10 = 3`).
> 2. P = 0.04. integral = 0.01, I = 0.00008. D = 0 (error unchanged). Output ≈ **0.040**.
> 3. **I**. P alone settles short (steady-state error). I keeps building until the error is truly zero.
> 4. 190 − 170 = **+20°** (no wrap needed). Or with −170: −170 − 170 = −340 → +360 = **+20°**. Same answer either way ✓.
> 5. **Zero.** Their Ki is 0, so they have no I term at all.

### Set D — Mixing and physics

1. base 0.6, pitch −0.1, roll 0, yaw 0. Motor values? Which end rises?
2. A 2 kg drone. Weight? Thrust needed to accelerate upward at 1 m/s²?
3. An airframe has 2:1 thrust-to-weight. Hover throttle?
4. Euler step: v = 3 m/s, a = −2 m/s², pos = 50 m, dt = 0.5. New v and pos?
5. Left motors 3 N each, right motors 3.3 N each, d = 0.163 m, I = 0.015. Angular acceleration? Which way does it roll?

> **Answers:**
> 1. M1 = M2 = **0.5** (front), M3 = M4 = **0.7** (rear). The rear rises → nose down.
> 2. Weight = 2 × 9.81 = **19.62 N**. F = ma → leftover needed = 2 × 1 = 2 N → thrust = **21.62 N**.
> 3. √(1 ÷ 2) = **0.707**.
> 4. v = 3 + (−2)(0.5) = **2 m/s**. pos = 50 + 2 × 0.5 = **51 m**.
> 5. τ = 0.163 × (6 − 6.6) = −0.098 N·m. α = −0.098 ÷ 0.015 = **−6.5 rad/s²** (flip: `÷ 0.015 ≈ × 66.7`). Negative roll: the right side is pushing harder, so the right side rises and it banks **left**.

### Set E — Navigation and frames

1. Leg from (0, 0) to (0, 50) (due north). Drone at (6, 10). Lookahead 18. Steering arrow?
2. Same situation, drone facing east (h = 90°). Forward and right errors?

> **Answers:**
> 1. u = (0, 1). proj = 10. carrot = 28 → point (0, 28). Arrow = (0 − 6, 28 − 10) = **(−6, 18)** (east, north): 6 m west, back to the line, and 18 m north, along it.
> 2. north = 18, east = −6. forward = 18·cos 90° + (−6)·sin 90° = **−6**. right = −18·sin 90° + (−6)·cos 90° = **−18**. "6 m behind me, 18 m to my left." Facing east, north is on the left ✓.

---

## Reference sheet

```text
TRIG
  split an arrow:      along = L·cos θ     sticks out = L·sin θ      (cos = along the line you measure FROM)
  steepness:           tan θ = sticks-out ÷ along = sin θ ÷ cos θ
  back to an angle:    θ = atan2(sticks-out, along)                  (never lose the quadrant)
  compass bearing:     atan2(east, north), then fmod(… + 360, 360)
  radians:             rad = deg × π/180 (0.01745)     deg = rad × 180/π (57.2958)
  tilt compensation:   tiltFactor = 1 / (cos roll · cos pitch), clamp [1, 2]

VECTORS
  length:              √(e² + n²)
  me → target:         target − me
  unit vector:         v ÷ |v|                       (guard against |v| ≈ 0)
  dot product:         a·b = aₑbₑ + aₙbₙ = |a||b| cos(angle)
  projection:          progress along a line = p · u   (u must be a unit vector)

CALCULUS (the computer version)
  derivative:          (new − old) ÷ dt              speedometer
  integral:            total += value × dt           odometer / running total

PID
  error = target − actual
  out = Kp·error + Ki·Σ(error·dt) + Kd·(Δerror ÷ dt), clamped
  anti-windup: clamp the integral at ±outMax/Ki
  angle errors: wrap into −180…+180

MIXING (M1 FL, M2 FR, M3 RL, M4 RR)
  M1 = base + pitch + roll − yaw        M2 = base + pitch − roll + yaw
  M3 = base − pitch + roll + yaw        M4 = base − pitch − roll − yaw

GPS
  north m = Δlat × 111,320               east m = Δlon × 111,320 × cos(lat)
  north = d·cos(bearing)                 east = d·sin(bearing)

PHYSICS
  a = F ÷ m          weight = m·g (g = 9.81)          thrust ∝ throttle²
  hover throttle = √(1 ÷ thrust-to-weight)
  v += a·dt, then p += v·dt
  torque = force × lever arm             α = τ ÷ I

FRAMES
  forward = n·cos h + e·sin h             right = −n·sin h + e·cos h

QUATERNIONS
  q = (cos ½θ, axis·sin ½θ), always length 1;  update: q += rate·dt, then normalize
```

---

## Study routine: 45 minutes, one lesson

```text
10 min  UNDERSTAND   Read the ELI5 and the picture. Close the file. Explain it out loud, like
                     you're teaching someone. If you can't, re-read the ELI5, not the formula.

10 min  REDRAW       On paper, from memory: redraw the lesson's picture and write its one-line
                     takeaway. Compare with the file.

15 min  CALCULATE    Do the worked example yourself without looking, then the "your turn."
                     Always finish with a sanity check (Pythagoras, units, "does the sign make sense?").

10 min  FIND IT      Open the linked source file. Find the line. Change a number in your head
                     and predict what the drone would do differently.
```

**One lesson a day beats five in one sitting.** The ideas need sleep in between to stick.

### How to know you've got it

- **Recognize:** you see `atan2(y, x)`, `a·b`, or `+= x * dt` in code and immediately know its job.
- **Rebuild:** you can re-derive `tiltFactor`, or the carrot, on paper from the pictures, without the file.
- **Transfer:** you can spot the same idea in a new place. For example, "`rpyDeg()` is just Lesson 1.8," or "that line uses an angle where it needs a rate" (Lesson 4.8).

---

## The whole course, in five sentences

1. **Trig splits arrows into pieces** (`cos` along, `sin` sticking out), and **`atan2` puts them back into an angle.**
2. **Vectors are arrows as numbers:** `target − me` says where to go, and the **dot product** says how much two directions agree.
3. **Calculus in code is two lines:** `(new − old) ÷ dt` and `total += value × dt`.
4. **PID turns error into push:** P for now, I for history, D for trend. The **mixer** spreads that push across four motors.
5. **Physics closes the loop:** `F = ma` and `τ = Iα`, integrated every millisecond, and the drone does it all again 200 times a second.
