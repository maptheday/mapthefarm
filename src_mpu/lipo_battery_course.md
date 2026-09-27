# The Drone Battery Course

*Everything a first-time drone builder needs to know about rechargeable batteries: what they are, why some catch fire, and the habits that keep you safe. Every lesson ends with a **⚖️ Compare** box showing how the four battery types differ.*

*Your drone uses **LiFePO4**: two [Pulse 3S 2100 mAh packs](https://www.boomerangrcjets.com/products/pulse-2100mah-3s-9-9v-25c-receiver-lifepo4-battery-xt60-connector), charged on an ISDT PD60.*

---

## Before you start

**Why this course exists.** Read any drone forum and you'll find two camps shouting at each other:

- *"LiPos are no more dangerous than your phone battery. Stop panicking."*
- *"My garage burned down. Use an ammo can for everything."*

**Both camps are right**, and this course shows you why:

> **Batteries rarely fail on their own. Almost every fire comes from one of a few mistakes. But when one does fail, it can fail fast and hot, so you also plan for the rare bad day. And some battery types fail far more gently than others.**

One forum comment sums up the whole course:

> *"The battery in your laptop has a computer making sure it's charged and discharged safely. **You are the computer for FPV batteries.**"*

Your phone and laptop have a tiny protection circuit inside (a "BMS", battery management system) that refuses to overcharge, over-drain, or overheat. Hobby batteries have **none of that**. Their only protection is **you**, plus your charger and your habits.

**How lessons work:**

| Beat | What it does |
|---|---|
| 🧸 **ELI5** | an everyday picture |
| 🔢 **Numbers** | the actual values that matter |
| ⚖️ **Compare** | how LiPo, LiFePO4, Li-ion, and NiMH differ on this topic |
| 🔋 **Your pack** | what it means for your 3S 2100 mAh LiFePO4 |
| ✍️ **Your turn** | one question, answer right below |

**The route:**

```text
 Part 1  Meet the batteries ........ the four types, cells, volts, energy, "C"
 Part 2  Why batteries catch fire .. thermal runaway, its 5 causes, which types resist it
 Part 3  Charging .................. the riskiest hour
 Part 4  Flying .................... fuel gauges, the 80% rule, crashes
 Part 5  Storing ................... storage levels, and where to keep them
 Part 6  Containers ................ bags vs. ammo cans (the forum fight, settled)
 Part 7  When it goes wrong ........ warning signs and what to do
 Part 8  Retiring a battery ........ damaged packs and disposal
 Part 9  Checklists ................ print these
 Part 10 Myths vs. facts + quiz
```

---

# Part 1 — Meet the batteries

## Lesson 1.1 — The four families

### 🧸 ELI5: four kinds of vehicles

| Type | Think of it as... | Where you've seen it |
|---|---|---|
| **LiPo** (lithium polymer) | a **race car**: fastest, lightest, but crashes badly | most FPV drones, RC cars and planes, newer airsoft packs |
| **Li-ion** (lithium-ion cylinders, like 18650/21700) | a **marathon runner**: goes the farthest, but can't sprint | phones, laptops, power tools, e-bikes, long-range drones |
| **LiFePO4** ("LiFe", lithium iron phosphate) | a **pickup truck**: heavier and slower, but very hard to wreck | **your drone**, RC receiver packs, some airsoft packs, home solar batteries |
| **NiMH** (nickel-metal hydride) | an **old tractor**: heavy and slow, almost impossible to hurt | your old airsoft stick packs, rechargeable AA batteries |

The first three are all **lithium** batteries: they store energy by moving lithium around. NiMH is an older, non-lithium chemistry.

### ⚖️ Compare: the big picture

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Energy for its weight | high | **highest** | medium | low |
| How hard it can push current | **highest** | low–medium | medium–high | medium |
| Fire risk if abused or crashed | **highest** | high (but tougher case) | **low** | very low |
| Needs careful habits? | **very** | very | yes, but forgiving | barely |
| Charge cycles before worn out | ~300–500 | ~500–1000 | **1000–2000+** | ~500–1000 |
| Good for your drone? | yes (most drones) | yes (long-range drones) | **yes, the safe choice** | ❌ too heavy |

**Takeaway:** *LiPo is the race car, Li-ion the marathon runner, LiFePO4 the pickup truck, NiMH the tractor. Your drone uses the pickup truck.*

---

## Lesson 1.2 — A cell is a sandwich

### 🧸 ELI5

Every battery is made of **cells**. A lithium cell is a sandwich: two "slices of bread" (the **electrodes**) with a sheet of very thin plastic in between (the **separator**), soaked in a liquid (the **electrolyte**) and sealed in a case.

**Charging** pushes lithium from one slice to the other, like winding up a spring. **Discharging** (flying) lets it flow back, and that flow powers your motors.

```text
      case
   ┌─────────────────────────────────┐
   │  ▓▓▓▓▓▓▓  (−) electrode          │
   │  ─ ─ ─ ─  separator (thin plastic, the only thing keeping + and − apart)
   │  ░░░░░░░  (+) electrode          │
   │    …many layers, rolled or stacked…
   └─────────────────────────────────┘
```

**The whole safety story lives in that separator.** It's thinner than a hair. If anything breaches it (a crash, overcharging, over-draining, heat), the + and − sides touch **inside** the cell. That internal short circuit is where fires start (Part 2).

### ⚖️ Compare: what the case is made of

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Case | **soft foil pouch** | **steel can** (cylinder) | pouch *or* can (A123-style packs use cans) | steel can |
| Crush resistance | poor: dents easily | good | pouch: poor; can: good | good |
| Weight of the case | lightest | heavier | varies | heavier |
| Shows damage by... | **puffing up** (easy to see) | usually nothing visible (the can hides it) | pouch puffs; can shows little | leaking or getting hot |

**Takeaway:** *Every cell is a sandwich with a hair-thin separator. Soft pouches (LiPo) are light but easy to damage; steel cans are tougher.*

---

## Lesson 1.3 — Volts: how "full" a cell is

### 🧸 ELI5

A cell's **voltage** works like a fuel gauge. For lithium batteries, though, **both ends are dangerous**: too full is bad, and too empty is bad. Each chemistry has its own safe window.

### ⚖️ Compare: the safe window, per cell

| Per cell | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| **Full** (never above) | 4.20 V | 4.20 V | **3.65 V** | ~1.45 V (charger decides) |
| **Storage** | 3.80 V | ~3.70 V | **3.30 V** | store charged; very forgiving |
| **Nominal** (the label) | 3.70 V | 3.60–3.70 V | **3.20–3.30 V** | 1.20 V |
| **Land now** (at rest) | 3.50 V | ~3.20 V | **3.00 V** | ~1.10 V |
| **Empty / damage below** | 3.00 V | ~2.75 V | **2.50 V** | ~0.90 V |

```text
 LiPo    DANGER │ 3.0 ────── 3.5 ── 3.8 ────── 4.2 │ DANGER
 LiFe    DANGER │ 2.5 ────── 3.0 ── 3.3 ───── 3.65 │ DANGER
```

⚠️ **Never use one chemistry's numbers (or charger setting) on another.** Charging your LiFe pack on a LiPo setting would push each cell to 4.2 V, far past LiFe's 3.65 V maximum.

**Takeaway:** *Every chemistry has its own safe window. Yours: 2.5 to 3.65 V per cell, stored at 3.3, landed by 3.0.*

---

## Lesson 1.4 — "3S": cells in a row

### 🧸 ELI5

One cell isn't enough voltage for your motors. So packs chain cells end to end, like batteries in a flashlight. **"S" means series** (cells in a row); **"P" means parallel** (side by side, for more capacity).

**The catch:** the cells in a pack can drift apart. One might sit at 3.30 V while another sits at 3.20 V. If you only watched the total, you'd never notice one cell drifting into danger. That's what the **balance lead** is for: the little white plug with **4 wires** (for 3S) connects to each cell, so a charger or checker can see every cell individually.

```text
   big XT60 plug ─────── carries the power (the total voltage)
   white balance plug ── 4 thin wires: lets a charger see cell 1, 2, and 3 separately
```

### ⚖️ Compare: what "about 10 volts" takes

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Cells for ~10–11 V | 3 (3S = 11.1 V) | 3 (3S = 10.8 V) | **3 (3S = 9.9 V)** | 8–9 cells (9.6–10.8 V) |
| Balance lead? | yes | yes | **yes** | no (NiMH chargers watch the whole pack) |
| Your old airsoft pack | a 7.4 V LiPo = 2S | — | a 9.9 V LiFe = 3S | an 8.4 V stick = 7 cells, 9.6 V = 8 cells |

🔋 **Your pack:** 3S LiFePO4 = **9.9 V** nominal, about **11.0 V** full, **7.5 V** empty.

**Takeaway:** *3S = three cells in a row. The balance lead lets you watch each cell, because one bad cell can hide inside a normal-looking total.*

---

## Lesson 1.5 — mAh and Wh: how big the tank is

### 🧸 ELI5

**mAh** (milliamp-hours) is the size of the tank. 2100 mAh = 2.1 amp-hours: the pack can supply 2.1 amps for one hour, or about 13 amps for 10 minutes.

**Wh** (watt-hours) is the **total energy** inside. It's the number that sets flight time, and how big a fire could be.

```text
 energy (Wh) = volts × amp-hours

 your 3S 2100 mAh LiFe:  9.9 V × 2.1 Ah = 20.8 Wh
 a 3S 5000 mAh LiPo:    11.1 V × 5.0 Ah = 55.5 Wh
 a phone battery:        ≈ 15 Wh      a laptop battery:  ≈ 50–100 Wh
```

### ⚖️ Compare: how heavy 21 Wh of energy is

The key difference between the types is **energy density**: how much energy each gram holds.

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Energy per kg (roughly) | 140–180 Wh | 180–240 Wh | **90–125 Wh** | 60–80 Wh |
| 21 Wh would weigh about | 130 g | 105 g | **168 g (your Pulse pack)** | 300 g |
| Flight time on your drone, same weight of battery | ~1.3× yours | ~1.6× yours | **1× (6–8 min)** | ~0.5× (too heavy to be useful) |

That's the trade in one table: **LiFe gives up some flight time for a much lower fire risk.** NiMH is safer still, but so heavy the drone would barely fly.

**Takeaway:** *Your pack holds ~21 Wh, about one and a half phones' worth. LiFe stores less energy per gram than LiPo or Li-ion, which is the price of its safety.*

---

## Lesson 1.6 — The "C" rating: how fast it can empty

### 🧸 ELI5

**C** measures how fast a battery can be drained, relative to its size. **1C** empties the whole tank in one hour; **2C** in half an hour; **60C** in one minute.

```text
 current = C rating × capacity (in amps)

 your pack:  1C  = 2.1 A    ← also the safe CHARGE rate (Part 3)
             25C = 52 A     ← max continuous DISCHARGE the label claims
             50C = 105 A    ← short bursts
```

Your four motors pull roughly **13 A while hovering** (an estimate), about a quarter of what the pack can give. Two honest notes: labels are often **exaggerated**, especially on cheap packs, and **charging C is not discharging C**. Charge at 1C unless the label says otherwise.

### ⚖️ Compare: how hard each can push

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Typical discharge rating | 30–120C (real: less) | **5–10C** | **20–30C** | ~10C |
| Enough for your drone's ~13 A hover? | easily | yes, for a big enough pack | **yes (25C = 52 A)** | yes |
| Good for racing punches? | **best** | poor | okay | poor |

This is why racing drones use LiPo and long-range cruising drones use Li-ion: one sprints, the other cruises. Your drone hovers and cruises gently, so LiFe's middle ground is plenty.

**Takeaway:** *1C = the pack's capacity in amps. Charge at 1C (2.1 A). Hovering uses only a fraction of your pack's discharge rating.*

---

## Part 1 checkpoint

- [ ] The four types, and one strength and one weakness of each
- [ ] Why the separator matters
- [ ] Your LiFe voltages: 3.65 / 3.3 / 3.0 / 2.5
- [ ] What "3S" means, and why the balance lead exists
- [ ] Your pack's energy (~21 Wh), and why LiFe is heavier per Wh
- [ ] What 1C is for your pack (2.1 A)

✍️ **Your turn:** a 4S 1500 mAh **LiPo**. Nominal voltage, full voltage, and 1C charge current? Then the same for a 4S 1500 mAh **LiFePO4**.

> **Answer:** LiPo: 4 × 3.7 = **14.8 V** nominal, 4 × 4.2 = **16.8 V** full, 1C = **1.5 A**. LiFe: 4 × 3.3 = **13.2 V** nominal, 4 × 3.65 = **14.6 V** full, 1C = **1.5 A** (C only depends on capacity).

---

# Part 2 — Why batteries catch fire

## Lesson 2.1 — Thermal runaway: the domino chain

### 🧸 ELI5

Picture a row of dominoes where each one, as it falls, also *heats up the next one*. Once the first falls, the rest can't be stopped. That's **thermal runaway**:

```text
 something breaches the separator
          ↓
 + and − touch inside → internal short → HEAT
          ↓
 heat breaks down more of the separator and chemicals → MORE heat
          ↓
 the electrolyte boils into gas → a pouch PUFFS up (a can builds pressure)
          ↓
 the case bursts (the "POP… hiss" people describe) → hot gas, often ignites
          ↓
 fire, several hundred °C, thick toxic smoke
          ↓
 neighboring cells heat up → they start their own runaway (a chain reaction)
```

Two things make a lithium fire different from a paper fire:
1. **Some chemistries partly feed themselves.** When overheated, LiPo and Li-ion chemistry releases some of the oxygen that keeps the fire going, so smothering it doesn't reliably work. It usually burns until its energy is spent (a minute or two).
2. **The smoke is toxic.** Don't breathe it.

### ⚖️ Compare: how each type behaves in runaway

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Heat needed to start runaway | lower | lower | **much higher** | doesn't do lithium-style runaway |
| Feeds its own fire? | yes | yes | **much less**: its chemistry holds onto its oxygen | no |
| What failure usually looks like | puff → pop → **flames** | pressure → **venting jet of fire** from the can end | **swells, gets hot, vents smoke; flames much less likely** | gets hot, may leak or vent |
| Chain reaction to other cells | likely | likely | less likely | rare |

🧸 **The dominoes again:** LiPo and Li-ion dominoes are close together and tip easily. **LiFePO4 dominoes are heavy and far apart.** It takes much more to start them, and even then they tend to fall slowly instead of racing down the line.

**Takeaway:** *Once runaway starts it can't be stopped, only contained. LiFePO4 needs far more abuse to start it, and rarely makes flames. That's why your drone uses it.*

---

## Lesson 2.2 — The five ways to start the dominoes

The most important lesson in the course. In the forum thread, a commenter points out that every fire story comes with a footnote: *"I discharged it too far." "It had some damage." "I tricked the charger."* Those footnotes are the causes:

| # | Cause | What physically happens | How you prevent it |
|---|---|---|---|
| 1 | **Overcharging** | lithium plates out as metal "whiskers" that pierce the separator | a **balance** charger, set to the **right chemistry**, with the correct cell count |
| 2 | **Over-discharging** | the cell's insides partly dissolve and re-form badly; the damage shows up **at the next charge** | land on time; unplug after flying; never force-charge a pack the charger refuses |
| 3 | **Physical damage** | a crash dents, bends, or punctures the cell and crushes the separator | protect the pack in the frame; inspect after every crash |
| 4 | **Heat** | a hot cell breaks down faster | don't charge hot packs; never leave them in a hot car |
| 5 | **Manufacturing defects** | a flaw from the factory | buy reputable brands; watch new packs closely at first |

**You control causes 1–4.** Cause 5 is rare. That's why the careful-user camp says fires "don't just happen", and why the other camp still plans for #5.

### A real story from that thread, decoded

> *"It had been discharged too far but I tricked the charger into trying to charge it. It almost had a normal charge cycle but one cell lagged a bit. Then I heard POP — hiss…"*

"Discharged too far" is **cause #2**. "Tricked the charger" means he overrode its safety refusal. "One cell lagged" was the damaged cell, visible on the balance plug. "POP — hiss" was the pouch bursting. **If a charger refuses a pack, it's protecting you.**

### ⚖️ Compare: how sensitive each type is to each cause

| Cause | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| 1. Overcharge | very sensitive | very sensitive | **tolerant-ish** (still ruins cells; a LiPo setting is the big danger) | tolerant (can overheat) |
| 2. Over-discharge | very sensitive | sensitive | sensitive for cell health, **low fire risk** | tolerant |
| 3. Crash damage | **very sensitive** (soft pouch) | tougher can | **low fire risk** | very tough |
| 4. Heat | sensitive | sensitive | **tolerant** | tolerant |
| 5. Defects | rare, serious | rare, serious | **rare, milder** | rare, mild |

**Takeaway:** *Overcharge, over-discharge, damage, heat, defects. You control the first four. LiFePO4 turns most of them from "fire risk" into "ruined battery".*

---

## Lesson 2.3 — Warning signs: puffing and friends

### 🧸 ELI5

A puffy battery is a warning light. The gas inside means some chemistry went wrong: step 4 of the domino chain started, even if only a little.

```text
 healthy pouch:   ▭▭▭▭▭▭  flat, firm, square edges
 puffed pouch:    ⬭⬭⬭⬭⬭⬭  swollen, soft, pillowy
```

**The rule:** a puffed or damaged pack **never gets charged again** and **never flies again**. Retire it (Part 8).

### ⚖️ Compare: how each type warns you

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Main warning sign | **puffing** (obvious) | few: cans don't puff; watch for heat, dents, a cell that drifts | pouch packs puff; **can-cell packs (like A123 types) show dents, heat, drifting cells** | heat, leaking, crusty terminals |
| What to check | shape, firmness | dents in cans, torn wraps, cell voltages | shape (if pouch), dents, **cell balance** | leaks, heat |

**Takeaway:** *Puffed, dented, or drifting = retired. Steel cans hide damage better than pouches, so check cell voltages too.*

---

## Lesson 2.4 — Why your drone uses LiFePO4

Put Parts 1 and 2 together, for a drone flying over someone's crops:

### ⚖️ Compare: the choice for your drone (same battery weight, ~170 g)

| | LiPo | Li-ion | **LiFePO4 (chosen)** | NiMH |
|---|---|---|---|---|
| Flight time | ~9–10 min | ~10–12 min | **~6–8 min** | ~3–4 min |
| Fire risk after a crash in a dry field | highest | high | **low** | very low |
| Forgiving of beginner mistakes | least | low | **fairly** | most |
| Cost for two packs | ~$40–60 | ~$60–90 | **~$80** | ~$40 |
| Works with your firmware and ESCs | yes | yes | **yes** | too heavy to fly well |

*(Flight times are rough estimates for your drone. Time your real flights.)*

**The trade:** a couple of minutes of flight time, for a battery that's far less likely to start a field fire. For a first build over a farm, that's the right call.

**Honest limits:** LiFe is **safer, not fireproof**. Every habit in this course still applies. They just protect a much more forgiving battery.

**Takeaway:** *LiFePO4 trades flight time for a much lower fire risk. Over someone's crops, that's the right trade.*

---

# Part 3 — Charging: the riskiest hour

Most battery fires start **while charging**, because charging is when a damaged or mishandled cell gets pushed hardest.

## Lesson 3.1 — Balance charging

### 🧸 ELI5

Three kids each have a cup under one tap. A simple charger watches only the *total* water and stops when the total looks right, even if one cup has overflowed. A **balance charger** watches each cup and slows down the one that's getting full, so all three stop at exactly the same line.

### 🔋 Your charger settings (ISDT PD60)

```text
 battery type:   LiFe            ⚠️ NOT LiPo: that setting charges to 4.20 V per cell,
                                    far past LiFe's 3.65 V
 cell count:     3S              (check the screen shows 3 cells before starting)
 mode:           Balance charge  (never plain "charge")
 current:        2.1 A           (1C for your 2100 mAh pack; about an hour)
```

**Always plug in both leads:** the XT60 (power) **and** the white balance lead.

### ⚖️ Compare: how each type is charged

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Charger setting | LiPo | Li-ion (LiIon) | **LiFe** | NiMH |
| Charges up to (per cell) | 4.20 V | 4.20 V | **3.65 V** | charger detects "full" by a tiny voltage dip |
| Balance lead used? | yes | yes | **yes** | no |
| Safe default current | 1C | 0.5–1C | **1C** (tolerates faster, but 1C is gentle) | 0.5–1C |
| Wrong-setting danger | LiHV setting overcharges | LiPo setting fine (same 4.2 V) | **LiPo setting badly overcharges** | a lithium setting won't charge it properly |

**Takeaway:** *Always balance charge, on the matching chemistry setting, at 1C, with both leads. For you: **LiFe · 3S · Balance · 2.1 A.***

---

## Lesson 3.2 — The charging routine

1. **Inspect.** Puffy, dented, torn, or damaged wires? **Don't charge.** Retire it.
2. **Check voltages** with the cell checker. Any LiFe cell **below 2.5 V**, or cells more than ~0.05 V apart? Something's wrong. Don't force it.
3. **Cool first.** After flying, wait ~15 minutes until the pack is no warmer than your hand.
4. **Place it well:** on a non-flammable surface (ceramic tiles or concrete pavers; tile stores sell cheap chipped "seconds"), nothing flammable nearby, in a battery bag or an **open-lid** ammo can.
5. **Keep the charger away from the pack,** using extension leads if needed, and **outside** any bag or can. If the pack fails, you don't want the charger (plugged into your wall) involved.
6. **Stay there.** Never charge while asleep or away. You're the BMS: you need to see, hear, and smell it.
7. **Done?** Unplug both leads.

### ⚖️ Compare: how strict the routine needs to be

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Stay in the room while charging | **must** | **must** | **should** (still the rule) | good practice |
| Fireproof surface and bag | **must** | **must** | **recommended** | not needed |
| Charge only when cool | **must** | **must** | **should** | should |

**Takeaway:** *Inspect, check, cool, tile, charger away from the pack, stay there. LiFe is forgiving, but keep the habits anyway.*

---

# Part 4 — Flying

## Lesson 4.1 — Voltage sag and the fuel gauge

### 🧸 ELI5

Turn on every tap in your house and the water pressure drops; turn them off and it comes back. A battery under heavy load **sags** the same way, then **recovers** at rest. So judge a battery by its **resting** voltage.

The bigger question is whether voltage tells you how much is **left**. That depends on the chemistry:

```text
 volts                LiPo / Li-ion: a slope                LiFePO4: flat, then a cliff
   │ ▇▇▇▇▆▆▆▅▅▅▄▄▄▃▃▂▂▁                 │ ▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▇▆▅▃▁
   │                                    │                    ↑ cliff
   └──────────── used ──►               └──────────── used ──►
   "3.7 V ≈ about half left"            "3.25 V could be 80% or 30% left"
```

### ⚖️ Compare: can the voltage be your fuel gauge?

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Voltage curve | steady slope | steady slope | **flat, then a sudden cliff** | fairly flat |
| Voltage works as a fuel gauge? | yes | yes | **no, only near empty** | poorly |
| Sag under hard throttle | small | **large** | small–medium | medium |
| Best fuel gauge | voltage + timer | voltage + timer | **timer + "mAh the charger put back"** | timer |

🔋 **Your pack:** trust the **flight timer** (your firmware's `MAX_FLIGHT_TIME_MS`) and the **mAh your charger puts back** after each flight. That number tells you how much the flight really used.

**Takeaway:** *Voltage sags under load. For LiPo and Li-ion it works as a fuel gauge; for your LiFe it's flat until the cliff, so trust the timer and the charger's mAh.*

---

## Lesson 4.2 — The 80% rule

**Never use more than about 80% of a pack's capacity.** The last 20% stresses the cells, and for LiFe it's exactly where the voltage cliff is.

### 🔋 Your pack

```text
 usable:          80% of 2100 mAh = 1680 mAh
 hover current:   roughly 13 A (estimate for a ~0.9 kg F450 on 9.9 V)
 flight time:     1.68 Ah ÷ 13 A ≈ 7.7 minutes → call it 6–8 minutes
```

Your firmware doesn't measure battery voltage. Its protection is the **5-minute flight limit** (`MAX_FLIGHT_TIME_MS`), which fits inside one pack with margin. **Leave it at 5 minutes** unless your charger shows you're consistently putting back well under 1680 mAh.

A good habit: after each flight write down the **flight time**, and after charging, **the mAh the charger put back in**.

### ⚖️ Compare: when to land

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Use at most | 80% | 80% (some long-range pilots go further) | **80%** | ~80–90% |
| Land at (resting, per cell) | 3.5 V | ~3.2 V | **3.0 V** | ~1.1 V |
| What happens if you push past it | cells damaged, fire risk at next charge | cells damaged | **sudden power drop (the cliff)**, cells damaged | power fades gradually |

**Takeaway:** *Use at most 80%. For your pack that's about 6–8 minutes; the 5-minute firmware limit fits inside it.*

---

## Lesson 4.3 — After a crash

A crash is cause #3. The pack can look fine and still be damaged inside.

1. **Look before you touch.** Smoke, hissing, or a sweet chemical smell? **Stay back.** Let it finish on the ground and stop the grass around it from catching.
2. **If it's calm:** unplug it right away.
3. **Inspect:** dents, bends, a torn wrap, puffing, damaged wires?
4. **Park it** on bare dirt or concrete, away from anything that burns, and **watch it for 15 minutes**. Damaged cells can go into runaway minutes later.
5. **Any visible damage → retire it.**

### ⚖️ Compare: crash behavior

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Survives a hard knock | poorly | fairly well (steel cans) | **fairly well** | very well |
| If damaged, likely outcome | **can ignite**, sometimes minutes later | can vent fire from a can | **usually heat and smoke** | heat, leaking |
| 15-minute watch after a crash | **essential** | **essential** | **do it anyway** | good practice |

**Takeaway:** *After a crash: unplug, inspect, isolate, watch 15 minutes. Damaged = retired, whatever the chemistry.*

---

# Part 5 — Storing

## Lesson 5.1 — Storage level

### 🧸 ELI5

A rubber band left stretched to the max for weeks gets tired and saggy. Lithium cells left **full** for a long time wear out the same way (a LiPo can even puff). Left **empty**, they slowly drain into the damage zone. **Half-full** is their most relaxed state. The charger's **storage mode** takes a pack there.

### ⚖️ Compare: storing

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Store at (per cell) | 3.80 V | ~3.70 V | **~3.30 V** | charged, or partly charged |
| Harm from sitting full for days | **high** (can puff) | medium | **low** | none |
| Loses charge on its own | slowly (~2–3%/month) | slowly | **slowly** | **fast** (~15–30%/month; "low self-discharge" types less) |
| Cold weather | loses capacity | loses capacity | **loses more capacity**; never charge below freezing | handles cold better |
| Check stored packs | monthly | monthly | **every month or two** | top up before use |

🔋 **Your pack:** not flying for a week or more? Run storage mode **set to LiFe** (~3.3 V/cell). For a few days, leaving it charged is fine. Store it cool and dry, **never** in a hot car.

**Takeaway:** *Store lithium packs half-full: LiPo 3.8 V, Li-ion ~3.7 V, your LiFe ~3.3 V. LiFe is the most forgiving about it.*

---

## Lesson 5.2 — Always unplug

Your drone's ESP32 and ESC keep drawing a tiny bit of current whenever the battery is plugged in, even when "off". Overnight, that can drain a pack below its empty line. That's cause #2, and the damage shows up at your next charge.

### ⚖️ Compare

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Damage from being drained overnight | cells ruined, fire risk at next charge | cells ruined | **cells ruined, lower fire risk** | usually recovers |

**Takeaway:** *Unplug the battery from the drone the moment you're done, whatever the chemistry.*

---

# Part 6 — Containers: the forum fight, settled

## Lesson 6.1 — What each container actually does

| Option | Good at | Weakness |
|---|---|---|
| **Nothing, in the open on tile, with you watching** | you see, hear, and smell trouble immediately | zero containment |
| **Battery bag** (sold as "LiPo bags") | cheap; slows a single small failure; keeps sparks in | quality varies wildly; won't reliably contain several packs; **slows** a fire, doesn't make it harmless |
| **Steel ammo can, rubber seal removed or vent holes drilled** | much stronger containment; many trust it for storage | **with the seal left in, it's a pressure bomb**; a closed lid hides warning signs |
| **Purpose-built box** (e.g., BAT-SAFE) | designed and tested for this | costs more |

⚠️ **Ammo can rule, not optional:** remove the rubber seal or drill vents **before** any battery goes in.

### ⚖️ Compare: how much container each type needs

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Charging | bag or open-lid can, on tile, **attended** | same | **bag on tile, attended** | on a table, attended |
| Storage | vented ammo can recommended | vented can recommended | **bag is fine; vented can is a nice extra** | a drawer |
| Transport to the field | bag or can | bag or can | **bag** | anything |

---

## Lesson 6.2 — Where the sensible answer lands

1. **Charging:** in the open (or in a bag, or an ammo can with the lid **open**), on tiles, charger outside the container, **with you in the room**.
2. **Storage:** at storage level, in a bag or **vented** ammo can, on tiles, away from anything that burns. A smoke detector nearby is a cheap extra.
3. **Transport:** in the bag.
4. **Don't let a container replace good habits.** The container is the seatbelt; the habits are the careful driving.

🔋 **For your two 21 Wh LiFe packs,** a decent battery bag on tiles plus the habits is plenty. A vented ammo can (~$20) is a fine extra if it gives you peace of mind.

**Takeaway:** *Charge where you can watch it. Store vented. The container is the seatbelt, not the driver.*

---

# Part 7 — When it goes wrong

## Lesson 7.1 — Warning signs

Stop charging or flying, and **unplug if it's safe to**, at any of these:
- **Puffing** or swelling
- **Hot** to the touch (warmer than your hand)
- **Hissing** or popping sounds
- A **sweet or chemical smell**
- Smoke

## Lesson 7.2 — If it catches fire

1. **Your safety first.** Don't grab a burning battery bare-handed. Don't breathe the smoke.
2. **If it's safe:** use long tongs or thick oven mitts to move the bag or can **outside**, onto concrete or dirt.
3. **Stop the spread:** use water, an ABC fire extinguisher, or sand on anything around it that's catching. A burning lithium pack usually burns until spent (a minute or two); your job is to stop it lighting anything else.
4. **If it's spreading, or you're unsure: get out and call 911.** A room is replaceable.
5. **Afterward:** ventilate well, and let the remains cool completely (at least an hour) before handling them.

### ⚖️ Compare: what "going wrong" looks like

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Typical bad day | puff → pop → **flames** | **jet of fire** from the can end | **heat and smoke, rarely flames** | heat, leaking |
| How long it burns | a minute or two, intensely | a minute or two, intensely | usually smoke; brief if it does ignite | rarely burns |
| Danger to the surroundings | high | high | **lower** | low |

**Takeaway:** *You can't save the battery. Protect people first, then the room. Get out if in doubt.*

---

# Part 8 — Retiring a battery

Retire a pack when it's **puffed, dented, punctured, has damaged wires, won't hold a charge, or has a cell that keeps drifting** from the others.

1. **Drain it** with the charger's discharge mode, somewhere safe, attended.
2. **Tape over** the XT60 and balance plug contacts with electrical tape.
3. **Recycle it:** many hardware and electronics stores have battery drop-off bins (look up "Call2Recycle" for locations). **Never** put a battery in household trash; it can be crushed in a garbage truck and start a fire.
4. Skip the "soak it in saltwater" advice you'll see online. It's slow, messy, and corrodes the terminals. Use the charger and a drop-off.

### ⚖️ Compare: disposal

| | LiPo | Li-ion | **LiFePO4 (yours)** | NiMH |
|---|---|---|---|---|
| Drain first | yes | yes | **yes** | helpful |
| Recycle, never trash | **yes** | **yes** | **yes** | yes |
| Lifespan before retiring | ~300–500 cycles | ~500–1000 | **1000–2000+** | ~500–1000 |

**Takeaway:** *Drain, tape, recycle. Never the trash. Your LiFe packs should last for years of flying.*

---

# Part 9 — Checklists for your LiFe packs (print these)

### Before charging
- [ ] Pack flat, firm, no dents, wires intact
- [ ] All 3 cells above 2.5 V, and within ~0.05 V of each other
- [ ] Pack cool (≈ room temperature)
- [ ] On tiles, nothing flammable nearby, in the bag
- [ ] Charger set to **LiFe · 3S · Balance · 2.1 A** (⚠️ not LiPo), screen shows 3 cells
- [ ] Both leads plugged in; charger sitting away from the pack
- [ ] I'm staying in the room

### After charging
- [ ] Unplug both leads
- [ ] Note the mAh the charger put back (tells you how much the last flight used)
- [ ] Not flying for a week? → storage mode (LiFe)

### Before flying
- [ ] Cell checker: all cells ~3.4–3.6 V at rest, all about equal
- [ ] Pack strapped down tight, inside the frame
- [ ] Fire extinguisher (or water and a shovel) at the field; fly over green grass or dirt

### After flying
- [ ] Unplug the battery from the drone immediately
- [ ] Note the flight time
- [ ] Cool 15 minutes before charging
- [ ] After any crash: inspect, isolate, watch 15 minutes

### Monthly
- [ ] Check stored packs: still ~3.3 V per cell, still undamaged

*(Flying a LiPo someday? Same lists, with LiPo numbers: charger **LiPo · 3S · 1C**, full 4.2 V, storage 3.8 V, land 3.5 V, and stricter about the bag and tiles.)*

---

# Part 10 — Myths vs. facts, and a quiz

## Myths vs. facts

| You'll hear... | The truth |
|---|---|
| "Batteries randomly explode." | Rare. Nearly all fires trace back to overcharge, over-discharge, damage, heat, or cheap packs. |
| "So I don't need any precautions." | Defects exist, and lithium fires are fast and hot. Plan for the rare case. |
| "All lithium batteries are equally dangerous." | No. **LiFePO4 is far more resistant to runaway** than LiPo or Li-ion. |
| "LiFePO4 can't catch fire at all." | It can still overheat and smoke if badly abused. Safer, not fireproof. |
| "Li-ion is safer than LiPo because it's in a metal can." | The can resists crushing, but the chemistry is similar. Once it fails, it can jet flame. |
| "A LiPo bag makes charging safe." | It slows a small fire. Only your habits make charging safe. |
| "Seal the ammo can so the fire can't get out." | **Dangerous.** A sealed can becomes a pressure bomb. Remove the seal or drill vents. |
| "Charge in a closed box so I don't have to watch it." | You lose your eyes, ears, and nose, your best warning system. Stay in the room. |
| "If the charger refuses a low pack, override it." | That refusal is protecting you. It's the footnote in the garage-fire story. |
| "Any lithium setting on the charger is fine." | Charging LiFe on the LiPo setting badly overcharges it. Match the chemistry. |
| "A slightly puffy pack is fine for one more flight." | Puffed = retired. |
| "NiMH is safest, so use it in the drone." | Safest, yes, but so heavy the drone would barely fly. |

## ✍️ Quiz (answers below; cover them first)

1. Rank LiPo, Li-ion, LiFePO4, and NiMH from most to least fire risk.
2. Your LiFe pack's full, storage, and land voltages per cell? And a LiPo's?
3. What should your charger be set to, and what's the one dangerous wrong setting?
4. Why can't voltage tell you how much is left in your LiFe pack mid-flight? What do you use instead?
5. Why must an ammo can's rubber seal be removed?
6. You crash in the field. The pack looks fine and isn't smoking. What now?
7. Why is a charger refusing a very low battery a good thing?
8. Why do racing drones use LiPo, long-range drones use Li-ion, and your drone use LiFePO4?
9. You won't fly for two weeks. What do you do with your packs?

> **Answers**
> 1. Most to least: **LiPo ≈ Li-ion** (Li-ion's can resists crushing, but the chemistry is similar) → **LiFePO4** → **NiMH**.
> 2. LiFe: **3.65 V** full, **3.30 V** storage, **3.00 V** land. LiPo: **4.20 / 3.80 / 3.50 V**.
> 3. **LiFe · 3S · Balance charge · 2.1 A**, both leads connected. Never **LiPo**: it would push each cell to 4.2 V, far above LiFe's 3.65 V.
> 4. LiFe's voltage stays **flat** for most of the flight, then drops off a cliff. Use the **flight timer** and the **mAh the charger puts back**.
> 5. A failing battery releases gas. Sealed in, pressure builds until the can bursts. Vents let the gas out.
> 6. **Unplug, inspect** for dents or swelling, set it on bare dirt, and **watch it for 15 minutes**. Any damage → retire it.
> 7. Charging an over-discharged pack is cause #2. The charger is refusing to start the dominoes.
> 8. Racing needs huge bursts of current (**LiPo** has the highest discharge). Long range needs the most energy per gram (**Li-ion**). Your drone flies over crops, so it needs the **lowest fire risk** that still flies well (**LiFePO4**).
> 9. Run **storage mode set to LiFe** (~3.3 V/cell), and store the packs cool, in the bag, on tiles.

---

## The whole course in six sentences

1. There are four battery families: **LiPo** (race car), **Li-ion** (marathon runner), **LiFePO4** (pickup truck), and **NiMH** (tractor). They trade energy against safety.
2. Lithium batteries fail through **thermal runaway**, and nearly always because of **overcharging, over-discharging, damage, or heat**: things you control.
3. **LiFePO4 needs far more abuse to go into runaway, and rarely makes flames.** That's why your drone uses it, at the cost of a few minutes of flight time.
4. **Balance charge on the matching chemistry setting** (yours: **LiFe · 3S · 2.1 A**), attended, on tile, and never override a charger that refuses a pack.
5. **Land on time, use at most 80%, store half-full, unplug after flying**, and retire anything puffed, dented, or drifting.
6. Containers are the seatbelt; your habits are the careful driving. **You are the battery's computer.**
