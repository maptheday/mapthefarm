# The Drone Math Course

*A 10-lesson tour of every piece of math actually running on the drone, in the order it matters. You've had Calc 1-3, but I'm assuming none of it is fresh — every lesson starts from scratch and builds up in small numbered steps, so you can go one line at a time and never hit a wall. Each topic follows the same shape: **why it matters** → **the lesson, one step at a time** → **a picture** → **a worked example** → **the actual code**. The topics also build on each other (2 feeds 3 feeds 4...), so read them in order the first time through.*

---

## Topic 1: Why any of this matters

**Why this impacts the drone:** A quadcopter has exactly one lever it can pull, 200 times a second: *"spin these 4 motors this hard."* Everything else, staying level, flying to a GPS point, following a fence line, coming home, is math translating "where I want to be" into those 4 numbers. Get a sign wrong and the drone flips instead of leveling. Get a formula wrong and it flies to Ohio instead of the back corner of a field. There is no "mostly right" in this code; the math either matches physical reality or the drone crashes.

**The lesson:** Think of the whole flight controller as one big function:

```
f(where I am, where I want to be) -> (motor1, motor2, motor3, motor4)
```

Every topic below is one piece of that function. By the end you'll be able to read `MotorController.hpp` and `QuadSim.h` top to bottom and know exactly what every line computes and why.

**Example problem:** None yet, this is the map. Topics 2-4 build the *steering* math (trig, vectors, PID). Topics 5-7 build the *navigation* math (GPS geometry). Topics 8-10 build the *physics* math (forces, rotation, simulation).

**In the code:** The whole pipeline lives across three files you'll see repeatedly:
- [`src/services/PID.hpp`](src/services/PID.hpp) — the "how hard do I push" formula
- [`src/services/NavMath.hpp`](src/services/NavMath.hpp) — the "where is the target" geometry
- [`lib/QuadSim/src/QuadSim.h`](lib/QuadSim/src/QuadSim.h) — the physics the drone actually obeys (used by the simulator, but it's the same math a real drone experiences)

---

## Topic 2: Trigonometry review — angles are the drone's whole vocabulary

**Why this impacts the drone:** Roll, pitch, yaw, GPS bearing, compass heading — every one of these is an *angle*, and the drone has zero ability to reason about angles except through `sin`, `cos`, `atan2`. If you don't have `sin`/`cos` cold, nothing else below will make sense. Take this one slow — everything else in the course leans on it.

**Picture it first:** Imagine the drone tilted over, and its thrust (the push from the propellers) as one arrow of a fixed length, pointing straight out through its belly:
```text
              thrust vector (fixed length, always 1 unit long)
                    /|
                   / |
                  /  | this side = sideways push
                 /   |
                /θ___|
          ------------------ true "up" (the direction gravity pulls against)
                this side = upward push
```
The arrow never changes length — tilting just changes how much of it points "up" vs. "sideways." That trade-off is the entire lesson. Now let's understand *why* sin and cos are the tools that measure it.

**The lesson, built up from the ground:**

**Step 1 — what an angle is even doing here.** The drone is either sitting flat or tipped over, and "how tipped" is a single number: the angle `θ`. Flat is `0°`, tipped fully onto its side is `90°`. That's all "tilt" is — one number. Everything in this topic is squeezing useful facts out of that one number.

**Step 2 — the one magic fact that makes trig exist.** Here's the idea nobody tells you plainly, and it's the *why* behind everything else:

> If two right triangles have the same angle, their sides are in the same *proportions* — even if one triangle is tiny and the other is enormous.

Draw a right triangle with the same corner angle the size of a postage stamp, and another the size of a barn. The barn's sides are way longer. But if you take "the short side ÷ the longest side" in each, you get the *exact same number* both times. The ratio doesn't care how big the triangle is — it depends **only on the angle**.

```text
   SMALL triangle              BIG triangle (2x bigger, SAME angle θ)

        /|                            /  |
     5 / |                        10 /   |
      /  | 3                        /    | 6
     /   |                         /     |
    /θ___|                        /θ_____|
       4                              8

   short ÷ long = 3 ÷ 5 = 0.6     short ÷ long = 6 ÷ 10 = 0.6
                       \_______________________/
                         SAME number — the angle θ sets the ratio,
                         the size of the triangle does not.
```

That's a gift, because it means you can measure those ratios *once* for each angle and reuse them on any triangle forever.

**Step 3 — so `sin` and `cos` are just names for two of those ratios.** Mathematicians measured them and made two lookup tables:
- `cos(angle)` = (the side *next to* the angle) ÷ (the longest side, the hypotenuse)
- `sin(angle)` = (the side *across from* the angle) ÷ (the hypotenuse)

```text
                        /|
        hypotenuse     / |
      (the longest    /  |
       side, always  /   |  opposite
       across from  /    |  (the side ACROSS
       the square  /     |   from angle θ)
       corner)    /θ_____|
                    adjacent
                 (the side NEXT TO angle θ)

        cos(θ) = adjacent ÷ hypotenuse    ->  "next-to  ÷ longest"
        sin(θ) = opposite ÷ hypotenuse    ->  "across   ÷ longest"
```

So `cos(30°) = 0.866` is not magic — it's just saying "in *any* right triangle with a 30° corner, the side next to that corner is 86.6% as long as the hypotenuse." `sin` and `cos` literally mean "go look up that ratio for this angle."

**Step 4 — why every trig picture uses a hypotenuse of exactly 1.** Ratios (fractions) are a little abstract. So watch what happens if we make the hypotenuse exactly `1`:
- `cos(angle)` = (side next to angle) ÷ 1 = *just the side next to the angle*
- `sin(angle)` = (side across from angle) ÷ 1 = *just the side across from it*

```text
      RATIOS (hypotenuse = 5)          LENGTHS (hypotenuse = 1)

           /|                                /|
        5 / | 3                            1/ | sin(θ)   <- the opposite side
         /  |                              /  |             now equals sin(θ)
        /θ__|                             /θ__|                itself
           4                              cos(θ)          <- the adjacent side
                                                             equals cos(θ) itself
      cos = 4/5, sin = 3/5             cos, sin ARE the sides
      (abstract fractions)            (concrete lengths you can point at)
```

Dividing by 1 does nothing, so the ratios *become the actual side lengths*. That's the whole reason we drew the thrust arrow as length 1 up top: it turns abstract ratios into concrete lengths you can literally point at in the picture.

**Step 5 — now map it onto the tilted drone.** In our picture the tilt angle `θ` is measured *from the straight-up line*. So:
- The side sitting *next to* `θ` runs along "up" → that's `cos(θ)` → **how much thrust still points up.**
- The side *across from* `θ` sticks out sideways → that's `sin(θ)` → **how much thrust got redirected sideways.**

Check it against common sense at the extremes:
- Flat, `θ=0°`: `cos(0°)=1` (100% of thrust still fights gravity), `sin(0°)=0` (nothing sideways). ✓ makes sense, it's not tipped.
- On its side, `θ=90°`: `cos(90°)=0` (no upward push left), `sin(90°)=1` (all of it is sideways now). ✓

```text
   θ = 0° (flat)        θ = 45° (leaning)     θ = 90° (on its side)

        ↑                      ↗                    →
        |                    /                     ————
      [drone]            [drone]                 [drone]

   up  = cos(0)=1.00    up  = cos(45)=0.71    up  = cos(90)=0.00
   side= sin(0)=0.00    side= sin(45)=0.71    side= sin(90)=1.00
   "all push is up"     "push splits evenly"  "all push is sideways"
```

**Step 6 — the consequence the drone lives and dies by.** As `θ` grows, `cos(θ)` shrinks: *tilting always steals some of your upward push.* A drone that leans over to fly somewhere is quietly giving up lift, so it starts to sink unless it throttles up.

```text
   LEVEL                        TILTED                     TILTED + FIX
   full lift, holds height      less lift, SINKS           throttle boosted 1/cos(θ)

        ↑ 1.0                        ↖ cos(θ)                    ↖↖ (bigger push,
        |                           ╱                            ╱   so the up-part
     [====]  ~ ~ ~ height        [====]                       [====]  is back to 1.0)
                                     ╲  drifting down            ~ ~ ~ height held
                                      ↓
```

That "throttle up by `1/cos(θ)`" is literally the tilt-compensation line of code in Topic 4 — it puts back exactly what the lean stole.

**Step 7 — running it in reverse: `atan2`.** Everything above goes "angle → sides." Sometimes the drone has the opposite problem: it knows the two sides (say "5 m north, 3 m east" to a waypoint) and needs the *angle*. That reverse lookup is arctangent. But there's a real gotcha, and understanding it is the point:

Plain `atan(y/x)` first divides `y` by `x` — and division **throws away the individual signs**. The point `(+1, +1)` (up-and-right) gives the ratio `1`. The point `(−1, −1)` (down-and-left) *also* gives the ratio `1`. Same ratio, opposite directions — plain `atan` literally cannot tell them apart, so it can only ever answer within a 180° range.

`atan2(y, x)` takes `y` and `x` *separately*, so it still sees both signs and knows which of the four quadrants you're really in.

```text
                      +y (north)
                        |
        (-1, +1)        |        (+1, +1)
        up-LEFT      \  |  /      up-RIGHT
                      \ | /
      -x (west) -------+-------- +x (east)
                      / | \
        down-LEFT    /  |  \    down-RIGHT
        (-1, -1)        |        (+1, -1)
                        |
                      -y (south)

   plain atan only sees the ratio y/x:
        (+1,+1) -> 1/1 = 1        (-1,-1) -> -1/-1 = 1      <- SAME number!
        so atan thinks up-right and down-left are identical (they're opposite!)

   atan2 keeps x and y apart, sees both signs -> always picks the right quadrant.
```

That's why every piece of navigation code uses `atan2` and never plain `atan` — it's the difference between "fly northeast" and accidentally "fly southwest."

**Example problem:** A drone is tilted 30° from vertical. Its motors are pushing 10 Newtons of total thrust straight through its own "up" axis. How much of that thrust is actually fighting gravity (pointing at true up), and how much is wasted pushing it sideways?
- Straight-up component: `10 * cos(30°) = 10 * 0.866 = 8.66 N`
- Sideways component: `10 * sin(30°) = 10 * 0.5 = 5.0 N`
- So tilting steals `cos(30°)` of your climbing power and gives you `sin(30°)` of it as sideways push. This exact trade-off is *how a drone moves at all* — it tilts on purpose to steal some "up" and turn it into "sideways."

**In the code:** [`MotorController.hpp:59-63`](src/services/MotorController.hpp#L59-L63) uses exactly this idea in reverse — since tilting steals climbing power, it multiplies the throttle back up by `1/cos(tilt)` to compensate:
```cpp
float tiltFactor = 1.0f / (cosf(rollRad) * cosf(pitchRad));
tiltFactor = constrain(tiltFactor, 1.0f, 2.0f);
out.baseThrottle = constrain(out.baseThrottle * tiltFactor, 0.0f, 1.0f);
```
And [`NavMath.hpp:22-27`](src/services/NavMath.hpp#L22-L27) (`gpsBearing`) uses `atan2` for exactly the "which direction am I really facing" reason above.

---

## Topic 3: Vectors — "how far and which way" as one object

**Why this impacts the drone:** The drone never thinks "go north" and "go east" as two separate problems solved twice. It thinks in one 2D vector: `(north, east)`, meters in each direction. Every steering decision the drone makes is vector math: add two vectors, find a vector's length, split a vector into components.

**The lesson, one step at a time:**

**Step 1 — a vector is just an arrow.** It packs two facts into one object: *how far* and *which way*. On the drone the arrow is `(east, north)` in meters — instead of tracking "12 m east AND 5 m north" as two separate chores, you carry one arrow that means both.

**Step 2 — the arrow's length is the straight-line distance.** `|v| = sqrt(x² + y²)`. That's nothing new — it's the Pythagorean theorem on the arrow's two sides (east leg and north leg). A `(12, 5)` arrow has length `sqrt(144 + 25) = 13` meters.

**Step 3 — a unit vector is that arrow shrunk to length exactly 1.** You get it by dividing the arrow by its own length: `u = v / |v|`. Shrinking throws away the "how far" (it's always 1 now) and keeps only the "which way." Think of it as a **pure compass direction** with no distance attached.

**Step 4 — the dot product measures "do these two arrows point the same way?"** The formula is just multiply-matching-parts-and-add: `a·b = ax·bx + ay·by`. You don't need to feel *why* yet, just what it tells you: a big positive number means the arrows point roughly the same way, zero means they're perpendicular (90° apart), negative means they point opposite ways.

**Step 5 — the one use we actually care about: the projection.** If `u` is a unit vector (a pure direction), then `v·u` answers "how far does arrow `v` reach *along* `u`'s direction?" Picture the sun straight overhead casting `v`'s shadow down onto the line `u` points along — the length of that shadow is `v·u`. This is called a **projection**, and it's the single most useful trick in the whole course: it lets the drone ask "how far along the fence line am I?" while completely ignoring how far it's drifted sideways.

**Picture it:**
```text
   north
     |
  10 +        * drone (30, 10)
     |       /|
     |      / | <- the "wasted" perpendicular part (drift off the line)
     |     /  |
   0 +----+---+------------------ east
     0   30 (shadow drone casts   100
             onto the fence line = the projection)

   The dot product pos·u only measures the horizontal shadow (30),
   never the vertical drift (10). That's the whole trick: projection
   throws away everything perpendicular to the line you care about.
```

**Example problem:** A drone is at position `(30, 10)` meters (east, north) relative to its launch point. The fence line runs from `(0,0)` straight to `(100, 0)` (due east). How far *along* the fence line has the drone already traveled, ignoring how far off the line it's drifted?
- The fence direction as a unit vector is `u = (1, 0)` (it only points east).
- Projection: `pos · u = (30)(1) + (10)(0) = 30`.
- Answer: 30 meters along the line — the drone's 10m of north drift doesn't count, because the dot product only measures the "along the line" component. That's the whole point of a projection: it throws away the perpendicular part.

**In the code:** This exact projection is the heart of the new line-following code, [`NavMath.hpp:46-70`](src/services/NavMath.hpp#L46-L70) (`lineFollowNorthEast`):
```cpp
float uE = segE / segLen, uN = segN / segLen;         // unit vector along the leg
float proj = posE * uE + posN * uN;                    // how far along the leg we are
```
That `posE * uE + posN * uN` line **is** the dot product from the lesson above, computed live, 10 times a second, to figure out how far around the fence the drone has traveled.

---

## Topic 4: PID controllers — calculus doing the flying

**Why this impacts the drone:** This is the single most important piece of math on the whole aircraft. It answers, continuously: *"I want to be at X, I'm actually at Y, how hard do I push right now?"* Altitude, roll, pitch, yaw, and GPS position are all controlled by one of these, six separate copies running simultaneously.

**The lesson, one step at a time:**

**Step 1 — everything starts with one number, the error.** `error = where I want to be − where I actually am`. Target altitude 30 ft, actually at 25 ft → error is +5 (I'm 5 ft too low, so push up). Every part of a PID looks only at this one number. Get comfortable here before moving on: P, I, and D are just three different questions asked about the *same* error.

**Step 2 — P (proportional): react to the error right now.** `P = Kp · error`. Twice as far off means twice as hard a push. `Kp` is just a knob for "how aggressive." P alone works, but it tends to settle a little *short* of the target and sit there forever, never quite arriving.

**Step 3 — I (integral): cure that "never quite arrives."** Keep a running total: every tick, add `error · dt` to a sum. If you've been a hair too low for a while, that sum quietly grows and grows until it's finally big enough to nudge you exactly onto target. That running total *is* the integral `∫ error dt` from Calc — the area piling up under the error curve. On the drone this is what secretly discovers "how much steady throttle it takes to cancel gravity," without anyone ever telling it what gravity is.

**Step 4 — D (derivative): don't overshoot.** Look at how *fast* the error is shrinking: `(error_now − error_last) / dt`. That's just the plain definition of a derivative (rise over run) before you take the limit. If the gap is closing fast, D pushes back to ease you in gently — exactly like lifting off the gas *before* the stop sign instead of slamming the brake at it.

**Step 5 — add them up and cap it.** `output = P + I + D`, then clamp it so the result can't ask a motor for more than 100% (or, for altitude, more than a safe trim range). That's the whole controller.

> Why "tiny steps" instead of real calculus? A computer can't do a true integral or derivative (there's no such thing as an infinitely thin slice in code), so it approximates: the integral becomes `sum += error · dt` (a Riemann sum, exactly what you did by hand in Calc 1) and the derivative becomes `(error_now − error_last) / dt`. Same ideas you know, just chopped into 5-millisecond ticks.

**Picture it:** the drone's altitude climbing toward a target of 30 ft, and what each term is "looking at" while it happens:
```text
  alt (ft)
   30 |·············target·············●━━━━━━━━  <- settled (small P, I holds it, D quiet)
      |                          ●━━━━╱
      |                     ●━━━╱      <- D term braking (error closing FAST)
      |               ●━━━╱
      |          ●━━━╱
      |    ●━━━━╱     <- P term pushing hard (big gap = big push)
      |━━━╱
   25 |
      +---------------------------------------> time
      ^ I term (small, steady background push)
        quietly grows every tick to supply the hover throttle
        that gravity demands, so P doesn't have to do it alone

   P = reacts to the gap right now.  I = remembers being low for a while.
   D = watches the SLOPE of the curve and eases off before overshoot.
```

**Example problem:** Altitude target is 30 ft, drone is at 25 ft (`error = 5`). Last tick's error was 6 ft. `dt = 0.005s` (200 Hz loop). Gains: `Kp=0.02, Ki=0.008, Kd=0.10`. Assume the accumulated integral so far is `0.4`.
- P term: `0.02 * 5 = 0.1`
- New integral: `0.4 + 5*0.005 = 0.425` → I term: `0.008 * 0.425 = 0.0034`
- Derivative: `(5 - 6)/0.005 = -200` → D term: `0.10 * -200 = -20`
- Total: `0.1 + 0.0034 - 20 = -19.9` — the D term dominates here because the error is closing *very* fast (6→5 in one 5ms tick, i.e. 200 ft/s closing rate), so the controller slams the brakes to avoid overshoot. This is exactly the clamp-and-tune problem the codebase's own comments describe (see below).

**In the code:** [`PID.hpp:21-41`](src/services/PID.hpp#L21-L41), line for line matching the formula:
```cpp
float error = target - actual;             // error(t)
float p = _kp * error;                     // Kp * error(t)
_integral += error * dt;                   // Riemann sum ≈ ∫ error dt
float i = _ki * _integral;
float derivative = (error - _lastError) / dt;  // ≈ d(error)/dt
float d = _kd * derivative;
return clamp(p + i + d, _outMin, _outMax);
```
And [`MotorController.hpp:18`](src/services/MotorController.hpp#L18) has a great real-world war story in its comment: the altitude P gain was originally `0.08`, which saturated the output clamp at just 6 ft of error and turned the controller into on/off bang-bang control (P term maxes out immediately, ignoring how far past 6ft the error is) — the fix was dropping P to `0.02` so the P term stays proportional across a wider range of error before clamping.

---

## Topic 5: The Haversine formula — distance on a sphere, not a flat map

**Why this impacts the drone:** GPS gives you latitude/longitude, degrees on a *sphere* (the Earth), not x/y on a flat plane. If you compute distance with plain Pythagorean theorem on lat/lon, you get answers that are wrong by a factor that changes depending on how far north you are — because a degree of longitude gets physically shorter as you move away from the equator.

**The lesson, one step at a time:**

**Step 1 — the tempting shortcut that's wrong.** Latitude and longitude are just two numbers, so it feels like you should subtract them and Pythagorean-theorem the result, like flat `(x, y)` map coordinates. Don't. The Earth is a ball, not graph paper.

**Step 2 — why it's wrong (the key intuition).** Lines of longitude all meet at the poles, so they *pinch together* as you go north. Near the equator a degree of longitude is about 111 km wide; up in North Carolina it's only ~93 km. So the same "0.001° east" is a different number of real meters depending on where you are. A flat subtraction has no idea about this and gives distances that are wrong by a factor that changes with latitude.

**Step 3 — the fix, in one sentence.** Measure the distance *along the curved surface of the sphere* instead of across a flat grid. That curved shortest path is called the **great-circle distance** (think of the arc a plane actually flies).

**Step 4 — the formula (recognize the pieces, don't derive it).** The haversine formula does exactly that:
```
a = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)
distance = 2R · atan2(√a, √(1-a))
```
`R` is Earth's radius (~6,371,000 m). You never derive this by hand — the thing to *notice* is that it uses **no new ingredients**: it's still just the `sin`, `cos`, and `atan2` from Topic 2, arranged to bend around a sphere. The `cos(lat1)·cos(lat2)` factor is literally the "longitude pinches as you go north" correction from Step 2, baked into the math.

**Picture it:** why you can't just subtract lat/lon like flat x/y coordinates:
```text
        FLAT (wrong for GPS)              CURVED EARTH (right)
      lat2 *-------* (far apart?)         lat2 *
           |        \                          |\
           |         \                         | \  <- the actual straight-line
      lat1 *----------* lon2                   |  \    "as the crow flies"
         (lon1)                          lat1  *---* lon2 distance is SHORTER
                                              (lon1)     than it looks on paper,
                                                          because longitude lines
                                                          squeeze together as you
                                                          move away from the equator.
   That's why gpsDistanceMeters can't just do sqrt(dLat^2 + dLon^2) --
   haversine bends the math around the sphere instead of a flat grid.
```

**Example problem:** Two points 0.001° of latitude apart at the equator (`R=6,371,000m`). `Δlat = 0.001° = 0.0000175 rad`. Since `Δlon=0`, the formula collapses: `a = sin²(Δlat/2) ≈ (Δlat/2)²` for small angles, so `distance ≈ R · Δlat = 6,371,000 * 0.0000175 ≈ 111.4 m`. This is why "111,320 meters per degree of latitude" (a constant you'll see reused in Topic 6) isn't a magic number — it falls straight out of this formula for small distances.

**In the code:** [`NavMath.hpp:13-19`](src/services/NavMath.hpp#L13-L19), `gpsDistanceMeters`, is this formula verbatim:
```cpp
float a = sin(dLat/2)*sin(dLat/2) + cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
return R * 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
```
This is what decides "have I reached the waypoint yet?" (compared against `WAYPOINT_ACCEPT_RADIUS_M`) on every single navigation tick.

---

## Topic 6: Bearing and the North/East split — turning "how far" into "which way, in components"

**Why this impacts the drone:** Knowing the *distance* to a waypoint isn't enough to fly there — you need a *direction*, and then you need that direction broken into "how much should I lean north-ish" and "how much should I lean east-ish," because those map directly onto the drone's pitch and roll controllers (which only understand one axis each).

**The lesson, one step at a time:**

**Step 1 — first, get a single direction.** Feed the two GPS points to `atan2` (Topic 2, Step 3) and it hands back a compass **bearing**: one angle, 0–360°, meaning "head *this* way to reach the target."

**Step 2 — but the drone can't act on a bearing directly.** It has no "fly at 60°" control. It can only lean forward/back (pitch, which moves it north/south) and lean left/right (roll, which moves it east/west). So a single bearing has to be *split* into a north amount and an east amount — two separate numbers the two controllers can each chew on.

**Step 3 — the split is the exact same triangle as Topic 2, just relabeled.** Distance is the hypotenuse, bearing is the angle, and the two legs are your answers:
```
north = distance · cos(bearing)
east  = distance · sin(bearing)
```
If you took Calc 2/3, this is precisely the "polar → Cartesian" conversion (`r, θ` → `x, y`) you did when integrating over polar regions. Same move, new labels.

**Picture it:**
```text
                    north
                      |
                      |        * waypoint
                      |       /|
                      |      / |
              100m,   |     /  | north = 100*cos(60) = 50
              bearing |    /   |
              60 deg  |   /    |
                      |  /60°  |
                      | /______|________ east
                    drone      east = 100*sin(60) = 86.6

   Same triangle as Topic 2, just relabeled: bearing is the angle,
   distance is the hypotenuse, north/east are the two legs.
```

**Example problem:** A waypoint is 100 m away at a bearing of 60° (60° clockwise from north). How much should the drone move north vs. east to get there?
- North: `100 * cos(60°) = 100 * 0.5 = 50 m`
- East: `100 * sin(60°) = 100 * 0.866 = 86.6 m`
- Sanity check with Pythagorean theorem: `√(50² + 86.6²) = √(2500+7500) = √10000 = 100` ✓, matches the original distance.

**In the code:** [`NavMath.hpp:30-34`](src/services/NavMath.hpp#L30-L34), `bearingToNorthEast`, is exactly this:
```cpp
float rad = radians(bearingDeg);
northM = distM * cos(rad);
eastM  = distM * sin(rad);
```
This feeds straight into the nav PIDs from Topic 4: north error drives the pitch PID, east error drives the roll PID (see [`MissionPhase.hpp`](src/phases/MissionPhase.hpp), where `northNavigationCorrection`/`eastNavigationCorrection` are called).

---

## Topic 7: Line-following — the carrot-on-a-stick projection, in a real flight bug

**Why this impacts the drone:** This is the fix for a real bug you saw on the Clover farm map: aiming straight at the *next* waypoint lets the drone cut the inside of every turn, bowing up to 33 meters off the fence line the farmer actually walked. The fix is Topic 3's projection, applied to real GPS legs.

**The idea in one sentence:** don't aim at the far waypoint — aim at a moving target that always sits *on the line*, a little ahead of you, like a carrot dangled just past a horse's nose.

**The lesson, one step at a time** (every step is just the vector tools from Topic 3):

**Step 1 — describe the leg and yourself as arrows from A.** The leg you're supposed to fly is `seg = B − A` (arrow from this corner to the next). Where you actually are is `pos = current − A` (arrow from this corner to the drone).

**Step 2 — get the leg's pure direction.** Shrink `seg` to length 1: `u = seg / |seg|`. Now `u` is "which way the fence runs," with the distance stripped off (Topic 3, Step 3).

**Step 3 — find how far along the leg you've gotten.** Project your position onto the leg's direction: `proj = pos · u` (Topic 3, Step 5 — the shadow trick). This is a single number: "I'm X meters down the fence," and it *ignores* how far you've drifted sideways.

**Step 4 — dangle the carrot.** Put the target a fixed lookahead distance *past* where you are, but still on the line: `carrot = u · (proj + lookahead)`, capped so it never runs past B. Because it's built from `u`, the carrot is *always exactly on the fence line*, never off to the side.

**Step 5 — steer at the carrot.** The direction to fly is just `carrot − pos`. Chase that instead of the far waypoint, and you naturally curve back onto the line and then ride it — you can never catch the carrot, so you just keep hugging the road it's on.

**Picture it:** why aiming at the waypoint bows the path, and how the carrot fixes it:
```text
   AIMING AT THE WAYPOINT (bows inward)   LINE-FOLLOWING (hugs the line)
                                                        B (waypoint)
   A                    B (waypoint)      A             |
   *~~~~___              *                *----+--------*
        \~~~___         /                 |    ^carrot (always ON the line,
         \    ~~~___   /                  |    |lookahead ahead of you)
          \        ~~~*  <- drone cuts    | drone aims HERE, not at B,
           \__________/    the corner,    | so it curves back onto the
        drone's actual     drifting up    | line instead of drifting off it
        curved path         to 33m off

   The carrot is like a horse-and-cart carrot: always a fixed distance
   ahead of you, ALONG the road, never off to the side. You can never
   catch it, so you just keep steering toward the road it's on.
```

**Example problem:** Fence runs from `A=(0,0)` to `B=(0,100)` (due north, 100m). Drone is at `(5, 40)` — 5m east of the line, 40m up it. Lookahead = 15m.
- `seg = (0,100)`, `|seg| = 100`, `u = (0, 1)`.
- `pos = (5, 40) - (0,0) = (5, 40)`.
- `proj = (5)(0) + (40)(1) = 40`.
- Carrot distance along line: `40 + 15 = 55` → carrot point `= (0,1)*55 = (0, 55)`.
- Steering vector = `(0,55) - (5,40) = (-5, 15)`: fly 5m west, 15m north. Notice the target is *on* the fence line (x=0), not off at some diagonal to a distant waypoint — that's what pulls the drone back onto the line instead of just angling toward the far end.

**In the code:** [`NavMath.hpp:46-70`](src/services/NavMath.hpp#L46-L70), `lineFollowNorthEast`, is precisely these five steps:
```cpp
float uE = segE / segLen, uN = segN / segLen;          // step 2: unit vector
float proj = posE * uE + posN * uN;                     // step 3: dot product = projection
float carrot = proj + lookaheadM;                       // step 4: carrot distance
float carrotE = uE * carrot, carrotN = uN * carrot;     // step 4: carrot point
eastM  = carrotE - posE;  northM = carrotN - posN;      // step 5: steering vector
```
`WAYPOINT_LOOKAHEAD_M = 18.0f` in [`FlightConfig.hpp`](src/state/FlightConfig.hpp) is the tunable "how far ahead does the carrot sit" — smaller hugs the line tighter but gets twitchier, bigger is smoother but cuts corners more. This single change took the worst off-line deviation from 33m down to 3.6m in real flight tests.

---

## Topic 8: Forces and Newton's Second Law — from motor spin to acceleration

**Why this impacts the drone:** Everything above decides *what the drone should do*. This topic and the next two are how the physics simulator (QuadSim) decides *what actually happens* when motors spin — the ground truth every flight controller has to obey, real or simulated.

**The lesson, one step at a time** — it's a little chain: *spin → force → acceleration → motion*.

**Step 1 — spin becomes force.** A propeller pushes harder the faster it spins, but not in a straight line: thrust grows with the *square* of the spin speed, `T = k · ω²`. (Double the spin, quadruple the push. It's squared because moving through air, drag and lift always scale with speed², not speed — same reason wind at 40 mph shoves you four times as hard as 20 mph.) `k` is just a fixed number for a given propeller.

**Step 2 — force becomes acceleration.** This is plain Newton: `F = ma`, rearranged to `a = F/m`. The `F` here is the *net* force — add up everything pushing on the drone: thrust (up), gravity (down), drag (against whichever way it's moving). Divide by mass, get acceleration.

**Step 3 — acceleration becomes motion (integrate twice).** Straight from Calc 1: acceleration is how fast velocity changes, and velocity is how fast position changes. So integrate `a` once to get velocity, integrate again to get position:
```
v = v0 + ∫a dt      (in code: v += a * dt)
x = x0 + ∫v dt      (in code: x += v * dt)
```

**Step 4 — why the code "cheats" with tiny steps.** The computer can't integrate a smooth curve, so it walks forward in tiny 5-millisecond steps: "at this instant acceleration is `a`, so over the next tick velocity changes by `a·dt`; new velocity moves position by `v·dt`; repeat." That stepping is **Euler's method** — the exact same "chop it into little pieces and add them up" trick as the PID integral in Topic 4.

**Picture it:** the free-body diagram QuadSim solves every physics tick:
```text
                    ↑ Thrust = k·ω1² + k·ω2² + k·ω3² + k·ω4²
                    |         (sum of all 4 rotors, each ∝ speed²)
              ┌─────●─────┐
              │  DRONE    │ ← drag (opposes velocity, grows as it speeds up)
              │  m=1.2kg  │ ←
              └─────●─────┘
                    |
                    ↓ Gravity = m·g = 1.2 * 9.81 = 11.77 N  (constant, always down)

   net force = Thrust - Gravity - Drag   ->   a = F/m   ->   integrate twice:
   a --(∫dt)--> v --(∫dt)--> position        (Euler's method: tiny steps of this)
```

**Example problem:** A drone has mass `1.2 kg`. Total thrust from 4 rotors is `13.0 N` straight up. Gravity is `9.81 m/s²`. What's the net vertical acceleration, and after `0.01s`, what's the new vertical velocity (starting from rest)?
- Net force: `13.0 - (1.2 * 9.81) = 13.0 - 11.77 = 1.23 N` (upward).
- Acceleration: `a = F/m = 1.23/1.2 = 1.025 m/s²`.
- New velocity after one 0.01s step: `v = 0 + 1.025*0.01 = 0.01025 m/s` upward — the drone is starting to climb.

**In the code:** [`QuadSim.h:127-142`](lib/QuadSim/src/QuadSim.h#L127-L142):
```cpp
T[i] = p_.kThrust * w * w;                    // T = k * omega^2, per rotor
float thrust = T[0] + T[1] + T[2] + T[3];     // total thrust
float az = (tw[2] - p_.dragZ * s_.vel[2]) / p_.mass - kGravity;  // a = F/m - g
s_.vel[2] += az * h;                          // v += a * dt  (Euler integration)
s_.pos[2] += s_.vel[2] * h;                   // x += v * dt  (Euler integration, again)
```
Notice `dragZ * vel` subtracted from thrust before dividing by mass — that's air drag, a force proportional to velocity, added into the same `F=ma` sum before integrating.

---

## Topic 9: Torque and rotational dynamics — the angular version of F=ma

**Why this impacts the drone:** A quadcopter doesn't just move, it *tips* on purpose to move, and it must resist tipping when it doesn't want to. That's rotational physics: torque, angular acceleration, and angular velocity, the rotational twin of everything in Topic 8.

**The trick to this whole topic:** rotation is just Topic 8 with every word swapped for its "spinning" twin. Force → torque. Mass → moment of inertia. Acceleration → *angular* acceleration. If you know `F = ma`, you already know the shape of everything here.

**The lesson, one step at a time:**

**Step 1 — the rotational F = ma.**
```
τ = I · α        (torque = moment-of-inertia × angular-acceleration)
```
Same sentence as Newton, just the spinning version: a twisting push (`τ`) makes something spin up faster (`α`), and heavier-to-spin things (`I`) resist it.

**Step 2 — what "torque" is on a drone.** A torque is a twisting push, and you make one with a see-saw: a force times its distance from the pivot. When the right-side rotors spin faster than the left, that thrust *difference*, multiplied by the arm length out to the rotor, is a torque that rolls the drone. `torque = (force difference) × (arm length)`.

**Step 3 — what "moment of inertia" (`I`) is.** It's rotational stubbornness — how hard the drone is to *start* or *stop* spinning. It's the spinning twin of mass. The one new wrinkle vs. plain mass: it depends on how far the weight sits from the spin axis (mass out at the arm tips resists spin far more than mass at the center — same reason a figure skater spins faster when they pull their arms in).

**Step 4 — the one 3D complication (you can mostly wave it off).** If the drone is *already* spinning on one axis and you try to spin it on another, the two interfere a little. That's the `ω × (I·ω)` term below — a cross product (Calc 3) that measures that interference. At gentle drone speeds it's tiny; it's in the code for correctness, but you don't need it for intuition:
```
I · ω̇ = τ - ω × (I·ω)
```

**Picture it:** the quadcopter from above, see-saw torque on the roll axis:
```text
                         FRONT
                  FL(spins CCW)   FR(spins CW)
                     ⟲ slower       ⟳ faster
                       \             /
                        \    d      /   <- lever arm from center
                         \ <----> /
                    ------●●●●●●●●------
                         /         \
                        /           \
                     ⟲ slower       ⟳ faster
                  RL(spins CCW)   RR(spins CW)
                         REAR

   Right rotors spin faster -> more thrust on the right side ->
   torque = (thrust difference) x (lever arm d) -> drone rolls LEFT,
   exactly like pushing down harder on the right end of a see-saw.
```

**Example problem:** Roll torque `τx = 0.05 N·m`, roll moment of inertia `Ix = 0.015 kg·m²`, no cross-coupling for simplicity (`gyroX=0`). What's the angular acceleration, and after `0.01s` starting from rest, what's the new roll rate?
- `α = τ/I = 0.05/0.015 = 3.33 rad/s²`
- After one step: `ω = 0 + 3.33*0.01 = 0.0333 rad/s` (about 1.9°/s) — the drone is starting to roll.

**In the code:** The lever-arm torque calculation, [`QuadSim.h:151-155`](lib/QuadSim/src/QuadSim.h#L151-L155):
```cpp
const float d = p_.armLength * 0.70710678f;             // lever arm (see-saw distance)
float tauX = d * ((T[0] + T[2]) - (T[1] + T[3]));        // left rotors minus right rotors, times arm
```
(`0.70710678 = 1/√2`, because on an X-frame the arm isn't aligned with the roll axis — this is `cos(45°)` from Topic 2, resolving the arm length into its roll-axis component.)

The full rotational equation, [`QuadSim.h:164-173`](lib/QuadSim/src/QuadSim.h#L164-L173), matches the lesson exactly, including the cross-product/gyroscopic term:
```cpp
// Euler's rigid body eqns: I*wdot = tau - w x (I*w)
float gyroX = wy * wz * (Iz - Iy);          // the cross-product term
float wdotX = (tauX - gyroX) / Ix;          // alpha = (tau - gyro term) / I
s_.omega[0] += wdotX * h;                    // omega += alpha * dt  (Euler integration again)
```

---

## Topic 10: Quaternions — rotation math without the trig blowing up

**Why this impacts the drone:** You could track "roll, pitch, yaw" as three separate angles, but that representation has a famous failure called **gimbal lock**, where two rotation axes align and you permanently lose a degree of freedom (the math can no longer tell two different orientations apart). Quaternions avoid this entirely and are the standard in every real flight controller.

**The lesson, one step at a time.** This is the scariest-*looking* topic and the one you can most safely treat as a black box — so let's go slow on the *why* and stay light on the algebra.

**Step 1 — the problem we're escaping.** The obvious way to store "which way is the drone tilted" is three angles: roll, pitch, yaw. It works... until it doesn't. Pitch the nose straight up 90° and the roll axis and yaw axis end up pointing the *same* direction — now two of your three controls do the same thing and you've silently lost one. That failure is **gimbal lock**, and it's why no real flight controller stores orientation as three angles.

**Step 2 — the fix, in plain words.** Any orientation, no matter how twisted, can be reached by picking *one* axis and rotating around it by *one* angle. A **quaternion** is just an efficient way to pack that "one axis + one angle" into 4 numbers `(w, x, y, z)`. Because there's only ever one axis, no two axes can ever collide — gimbal lock is impossible by construction. (Under the hood it's a cousin of complex numbers, `w + xi + yj + zk`, but you never need that to use it.)

**Step 3 — the drone only ever does two things with it, and you already know both patterns:**

  **(a) Push it forward in time as the drone spins.** Given the current spin rate `ω`, the quaternion changes a little each tick:
```
q̇ = 0.5 · q ⊗ (0, ωx, ωy, ωz)          (⊗ just means "combine two rotations")
```
  You don't compute this by hand — the point is it's the *same Euler step from Topics 8–9*: take a rate of change, multiply by `dt`, add it on. Just applied to a 4-number object instead of a single number.

  **(b) Ask "which way is my thrust really pointing?"** The drone knows thrust points "up" *in its own tilted frame*; it needs that in *world* terms (how much is truly up vs. sideways). Rotating a vector by a quaternion answers that — the code uses a standard formula for it, and its output is exactly the thrust arrow that feeds Topic 8's `F = ma`.

That's the whole practical toolkit. You can treat the 4-number bookkeeping as a sealed box labeled "orientation that never gimbal-locks," and just trust the two operations above.

**Picture it:** why 3 angles (roll/pitch/yaw) can betray you, but a quaternion never does:
```text
   ROLL/PITCH/YAW (can gimbal-lock)        QUATERNION (always 1 clean axis)
        yaw axis                                    single spin axis
           |                                              \
       ----+---- pitch axis                                \  the drone's
           |                                                 \ orientation is
        roll axis                                             "rotate by angle
                                                                 θ around THIS
   Pitch the nose up 90°: now roll and                          one axis" --
   yaw axes point the SAME direction --                         no axis can
   you've LOST an axis of control                               ever collide
   (gimbal lock). 3 separate angles                             with another,
   can collapse into each other.                                so no lock.
```

**Example problem:** Rather than grind the full quaternion algebra by hand (it's mechanical, not conceptual), the concept to hold onto: a quaternion's `w` component is close to 1 when the rotation is small, and its `x,y,z` components are approximately `(half-angle)·(rotation axis)` for small rotations. So a tiny roll rate `ωx = 0.1 rad/s` held for `dt=0.01s` produces a quaternion update where `x ≈ 0.5 * 0.1 * 0.01 = 0.0005` — a very small nudge to the `x` component per tick, which is exactly why you integrate it thousands of times a second rather than trying to jump straight to a final orientation.

**In the code:** The integration step, [`QuadSim.h:175-185`](lib/QuadSim/src/QuadSim.h#L175-L185):
```cpp
// integrate quaternion: qdot = 0.5 * q (x) (0, omega)
float dw = 0.5f * (-x0 * ox - y0 * oy - z0 * oz);
float dx = 0.5f * ( w0 * ox + y0 * oz - z0 * oy);
...
w0 += dw * h; x0 += dx * h; ...             // Euler integration, same pattern every time
float nrm = std::sqrt(w0*w0 + x0*x0 + y0*y0 + z0*z0);
s_.quat[0] = w0 / nrm; ...                   // re-normalize back to length 1
```
That last renormalization step exists because Euler integration is an *approximation* — each tiny step drifts the quaternion's length away from exactly 1, so it's rescaled back every tick (this is the numerical-methods lesson from Calc: approximation methods accumulate error, and you correct for it).

The vector-rotation sandwich, [`QuadSim.h:189-198`](lib/QuadSim/src/QuadSim.h#L189-L198) (`rotateBodyToWorld`), is what turns "thrust points along my body's up axis" into "thrust points *this* way in the real world" — the answer this function gives is exactly the `tw[3]` vector fed into Topic 8's `F=ma` equation. This is the thread that ties the whole course together: angles (2) → vectors (3) → PID steering (4) → GPS geometry (5-7) → forces (8) → torque (9) → orientation (10) → and orientation feeds straight back into Topic 8's force calculation, closing the loop the same way the real drone closes it, 200 times a second.

---

## Where to go from here

Read the files in this order and you'll recognize every formula: [`PID.hpp`](src/services/PID.hpp) → [`NavMath.hpp`](src/services/NavMath.hpp) → [`MotorController.hpp`](src/services/MotorController.hpp) → [`QuadSim.h`](lib/QuadSim/src/QuadSim.h). Every one of them is short (under 250 lines) specifically so this kind of read-through is possible in one sitting.
