#!/usr/bin/env python3
"""
ON-ESP scenario: MAX FLIGHT TIME -> LAND WHERE IT IS.

Boot-selects the `timeout` scenario, which shortens the max-flight-time limit to
~18 s. The drone takes off and starts the mission; when the clock runs out the
failsafe must land right there -- NOT fly home, because on a big field the trip
back could cost more battery than is left. (Replaces edge_max_flight_timeout.)
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="timeout", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[timeout] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    # when did the landing start?
    land_t = next((x["t"] for x in s if x["phase"] == "LANDING"), None)
    print(f"[timeout] phases: {' -> '.join(phases)}   LANDING began at "
          f"{'%.0fs' % land_t if land_t is not None else 'never'}")
    esp_sim.save(data, args.out)
    # The time limit (~18 s in) should land it right there: no RTL at all.
    ok = (land_t is not None and land_t >= 15.0 and "LANDED" in phases
          and not any(p.startswith("RTL") for p in phases))
    print("[timeout] " + ("PASS (max flight time -> landed where it was)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="max_timeout.json")
    ap.add_argument("--wait", type=float, default=180.0)
    sys.exit(run(ap.parse_args()))
