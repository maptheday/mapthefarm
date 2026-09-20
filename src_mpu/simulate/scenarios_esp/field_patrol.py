#!/usr/bin/env python3
"""
ON-ESP sim scenario: FIELD PERIMETER PATROL.

Flies the farmer's whole fence line autonomously on the ESP (QuadSim physics in
the real flight loop), pulls the flight log back, checks it flew tightly, and
saves the JSON the Clover / Map-the-Farm replay reads.

The `sim` firmware must already be flashed (run_hil.sh does that once):
  ~/.platformio/penv/bin/pio run -e sim -t upload

Run directly:
  simulate/.venv-rotorpy/bin/python simulate/scenarios_esp/field_patrol.py \
      --port /dev/cu.usbmodem14101 --out field_patrol.json
"""
import argparse, math, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="full", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[field_patrol] NO SAMPLES -- is the `sim` build flashed? try a longer --wait.")
        return 1

    # Mission visits corners 2..8 then back to 1 => corner indices 1..7, 0.
    order = [1, 2, 3, 4, 5, 6, 7, 0]
    mission = [x for x in s if x["phase"] == "MISSION"]
    worst_corner = 0.0
    print("[field_patrol] closest approach to each corner:")
    for wp, ci in enumerate(order):
        cn, ce = data["corners"][ci]["n"], data["corners"][ci]["e"]
        best = min((math.hypot(x["north"] - cn, x["east"] - ce) for x in mission), default=1e9)
        worst_corner = max(worst_corner, best)
        print(f"   wp{wp} corner{ci + 1}: {best:.1f} m")

    final = s[-1]
    home_dist = math.hypot(final["north"], final["east"])
    seq = list(dict.fromkeys(x["phase"] for x in s))
    print(f"[field_patrol] phases: {' -> '.join(seq)}")
    print(f"[field_patrol] max dist out: {max(x['dist'] for x in s):.0f} m   "
          f"lands {home_dist:.1f} m from home   worst corner {worst_corner:.1f} m")

    esp_sim.save(data, args.out)

    # Pass/fail: flew the mission, hit every corner tightly, landed near home.
    ok = ("MISSION" in seq and "LANDED" in seq and worst_corner < 6.0 and home_dist < 10.0)
    print("[field_patrol] " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="field_patrol.json")
    ap.add_argument("--wait", type=float, default=320.0)
    sys.exit(run(ap.parse_args()))
