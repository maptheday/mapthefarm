#!/usr/bin/env python3
"""
ON-ESP scenario: MANUAL FLIGHT.

Boot-selects the `manual` scenario: once hovering, the firmware switches to MANUAL
mode and pushes the pitch stick forward for a few seconds, so the real controller
flies the drone against the on-chip physics. Expect the drone to enter MANUAL and
actually move. (Replaces the old manual_flight_test.)
"""
import argparse, math, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="manual", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[manual] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    man = [x for x in s if x["phase"] == "MANUAL"]
    moved = max((math.hypot(x["north"], x["east"]) for x in man), default=0.0)
    print(f"[manual] phases: {' -> '.join(phases)}   moved {moved:.1f} m under manual control")
    esp_sim.save(data, args.out)
    ok = ("MANUAL" in phases and moved > 3.0)
    print("[manual] " + ("PASS (MANUAL mode flew the drone)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="manual_flight.json")
    ap.add_argument("--wait", type=float, default=60.0)
    sys.exit(run(ap.parse_args()))
