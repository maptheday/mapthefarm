#!/usr/bin/env python3
"""
ON-ESP scenario: LAND SWITCH -> LAND WHERE IT IS.

Boot-selects the `land` scenario: the drone takes off to its hover, then the
sim flips the LAND switch (the same crsfHandleLand() the real radio calls).
Expect a gentle landing right there: no mission, no return-home.
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="land", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[land] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    print(f"[land] phases: {' -> '.join(phases)}")
    esp_sim.save(data, args.out)
    ok = ("HOLD" in phases and "LANDING" in phases and "LANDED" in phases
          and "MISSION" not in phases and not any(p.startswith("RTL") for p in phases))
    print("[land] " + ("PASS (LAND switch -> landed where it was)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="land_switch.json")
    ap.add_argument("--wait", type=float, default=120.0)
    sys.exit(run(ap.parse_args()))
