#!/usr/bin/env python3
"""
ON-ESP scenario: LOW BATTERY -> LAND WHERE IT IS.

Boot-selects the `lowbatt` scenario: the sim's fake LiFe pack starts only 25%
charged, but the firmware assumes every pack is full when plugged in. So the
estimated-mAh gauge thinks all is well -- this tests the VOLTAGE backup gauge:
as the pack reaches LiFe's voltage "cliff", the battery failsafe must land the
drone right where it is (no flying home), before the mission can finish.
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="lowbatt", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[lowbatt] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    land = next((x for x in s if x["phase"] == "LANDING"), None)
    print(f"[lowbatt] phases: {' -> '.join(phases)}")
    if land is not None:
        print(f"[lowbatt] landing began at {land['t']:.0f}s, "
              f"{land.get('cell_v', 0):.2f} V/cell, {land.get('mah', 0):.0f} mAh used (est.)")
    esp_sim.save(data, args.out)
    # Landed where it was: LANDING without any RTL, and before finishing the
    # mission (HOVER_SETTLE only follows the last waypoint).
    ok = ("MISSION" in phases and "LANDING" in phases and "LANDED" in phases
          and not any(p.startswith("RTL") for p in phases)
          and "HOVER_SETTLE" not in phases)
    print("[lowbatt] " + ("PASS (low battery -> landed where it was)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="low_battery.json")
    ap.add_argument("--wait", type=float, default=240.0)
    sys.exit(run(ap.parse_args()))
