# ESC Code — Why We Did What We Did

Notes written for someone who is learning as they build. No assumed knowledge.

---

## Start here: what is actually happening when a drone flies?

A drone has four motors. Each motor spins a propeller. The faster the propeller spins, the more lift it creates. To go up, all four spin faster. To tilt left, the right side spins faster than the left. That's it — the entire flight controller is just a program that decides how fast each of those four motors should spin, many times per second.

The ESP32 is the brain. But the ESP32 can't spin motors directly — it's just a tiny computer chip. It runs on 3.3 volts and can barely power an LED. Motors need serious power from a battery.

So there's a middleman: the **ESC** (Electronic Speed Controller). One per motor. The ESP32 sends the ESC a signal saying "spin at 40% power," and the ESC figures out how to actually deliver that from the battery to the motor. Think of the ESP32 as the manager shouting orders, and the ESC as the worker actually doing the physical job.

---

## What is DShot600?

DShot600 is the language the ESP32 uses to talk to the ESC over a single wire.

It works by sending a number between 0 and 2047:
- 0 = stop
- 48 = minimum throttle
- 2047 = full throttle

But it can't just send the number "500" like a text message. Instead it converts that number into a pattern of electrical pulses — high voltage and low voltage, very rapidly. The ESC reads those pulses and decodes them back into the number.

DShot600 does this 600,000 times per second (that's what "600" means). Each individual pulse is extremely short — around 1–2 microseconds. That's why regular code can't do it reliably — the CPU might get distracted by something else mid-pulse and ruin the timing.

---

## Why can't we just use regular code to send pulses?

Imagine you're tapping out Morse code on a table, and someone keeps tapping you on the shoulder mid-tap to ask you something. Your timing gets ruined.

That's what happens if the CPU tries to send DShot pulses itself — WiFi interrupts, FreeRTOS task switches, anything can pull the CPU away mid-pulse and corrupt the signal.

The ESP32 has a hardware module called **RMT** built specifically for this. You hand it the pattern of pulses you want to send, and it sends them perfectly on its own without involving the CPU at all. Like giving your Morse code to a machine that taps it out perfectly every time, while you go do other things.

---

## What is RMT?

RMT stands for "Remote Control Module." It was originally designed for sending TV remote control signals, but it works perfectly for DShot too — both are just patterns of timed pulses.

You set it up once, give it a pulse pattern, and it handles the rest in hardware. The CPU is completely free while RMT is transmitting.

---

## Setting up RMT — what is `RMT_DEFAULT_CONFIG_TX`?

To use RMT, you have to fill out a configuration form with about 10 fields — what pin to use, what mode, what clock speed, whether to loop, and so on. Most of those fields are always the same and you'd never change them.

`RMT_DEFAULT_CONFIG_TX(pin, channel)` is a shortcut that pre-fills all those boring fields for you. Like ordering a burger and saying "the usual" — the only things that actually change are which pin and which channel you're using.

After that shortcut, we only change one thing ourselves: the clock speed.

---

## What is `clk_div = 1` and why does it matter?

The ESP32's internal clock ticks 80 million times per second. RMT uses this clock to measure how long each pulse should be.

`clk_div` lets you slow that clock down before RMT uses it. Setting it to `2` would halve it to 40 million ticks per second. Setting it to `4` would quarter it, and so on.

We set it to `1` — which means don't slow it down at all, use the full 80 million ticks per second. We want maximum precision because DShot600 pulses are extremely short and need to be timed very accurately.

---

## What are those magic numbers `{190, 1, 76, 0}` and `{95, 1, 171, 0}`?

Remember how DShot sends numbers as patterns of pulses? Each individual bit (a 1 or a 0) is represented by a pulse of a specific length:

- To send a `1` bit: hold the wire HIGH for 190 clock ticks, then LOW for 76 ticks
- To send a `0` bit: hold the wire HIGH for 95 clock ticks, then LOW for 171 ticks

The ESC reads those timings and says "that was a 1" or "that was a 0."

These numbers are defined by the DShot600 spec and never change. So instead of calculating them every single loop, we just hardcode them. Faster and clearer.

---

## Why `false` in `rmt_write_items(..., false)`?

The last argument answers the question: "after handing the pulse pattern to RMT, should I wait here until it finishes sending?"

- `true` = yes, wait here until RMT is done
- `false` = no, hand it off and immediately move on

We use `false` — fire and forget. RMT handles the sending on its own in hardware. There's no reason to stand there waiting.

This matters because our physics loop runs 200 times per second. We can't afford to waste time waiting for each ESC to finish when the RMT hardware can do it without us.

---

## Why one ESC instance per motor?

```cpp
EspESC esc1, esc2, esc3, esc4;
```

We could have written one big ESC class that controls all four motors at once. But then when something goes wrong, you're digging through that big class trying to figure out which motor it's talking about.

Instead, `esc1` is motor 1. Full stop. `esc3` is motor 3. If motor 2 is misbehaving, you look at `esc2`. Nothing to dig into, nothing to track down.

---

## What does `constrain(m1, 0.0f, 1.0f)` mean?

Our PID math produces motor values — numbers that represent how fast each motor should spin, between 0.0 (off) and 1.0 (full). But the PID math doesn't know about those limits. When corrections are large, it can spit out numbers like `1.3` or `-0.2`.

Those numbers don't make sense for a motor. There's no "130% throttle."

`constrain` is a guardrail. If the number is above 1.0, snap it down to 1.0. If it's below 0.0, snap it up to 0.0. Like a thermostat that won't let you set the temperature below 60° or above 90° no matter how far you turn the dial.

We always do this before sending anything to the ESC.

---

## Why is 0–47 off limits for throttle?

DShot reserves the lowest numbers (0–47) for special commands — things like "spin in reverse" or "beep the motor." They're not throttle values.

Real throttle starts at 48. So if our math accidentally produced a value of `10` and we sent it, the ESC wouldn't think "very low throttle" — it would think we were sending it a special command.

Our code catches this:

```cpp
if (throttle > 0 && throttle < 48) throttle = 48;
```

If the number sneaks into that danger zone, bump it up to 48. Zero is fine — that just means stop completely.

---

## Why did we remove the IESC interface?

The original flight controller code was designed to run in two places: on the real ESP32 hardware, and also on a Mac for testing without any physical hardware. That meant we needed two versions of the ESC code — a real one and a pretend one — and an `IESC` interface that let the rest of the code use either one without caring which.

In this project we only ever run on real hardware. There's no Mac simulator. So that whole swapping system has nothing to do. We removed it and just wrote the class directly — one less file, one less thing to read and understand.

---

## Does the ESC remember the last command forever?

No — and this is an important gotcha.

The ESC does not hold the last command indefinitely. Most ESCs have a timeout — if they stop receiving packets after around 250ms, they assume something went wrong (lost signal, crashed drone, dead controller) and cut the motors as a safety measure. Imagine a drone flying away and losing signal — you want the motors to stop, not keep spinning forever at whatever they were last told.

This means our physics loop running at 200Hz isn't just about accuracy. It's also keeping the ESC "fed" with a continuous stream of packets. As long as we keep sending `write(0.5)` every 5ms, the ESC stays at 50%. The moment we stop sending anything, the ESC starts counting down its timeout and cuts power.

Think of it like a dead man's switch on a train — the driver has to keep their hand pressed on it the whole time. Let go and the train stops. Our loop is the hand on the switch.

---

## Why does disarm send a signal every loop instead of just once?

This is why `flightEnabled = false` in our code calls `disarm()` every single loop iteration rather than just once:

```cpp
} else {
  esc1.disarm(); esc2.disarm(); esc3.disarm(); esc4.disarm();
}
```

It's not sending "stop" once and walking away — it's actively sending "stop, stop, stop, stop..." at 200Hz. This keeps the ESC in a known, confirmed-stopped state rather than just letting it time out into whatever it decides to do on its own.

Actively sending zero is safer than silence.