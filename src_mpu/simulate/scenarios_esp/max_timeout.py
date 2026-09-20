#!/usr/bin/env python3
"""
ON-ESP scenario: MAX FLIGHT TIME -> RTL.

Boot-selects the `timeout` scenario, which shortens the max-flight-time limit to
~18 s. The drone takes off and starts the mission; when the clock runs out the
failsafe must force Return-To-Launch and land. (Replaces edge_max_flight_timeout.)
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="timeout", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[timeout] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    # when did RTL start?
    rtl_t = next((x["t"] for x in s if x["phase"].startswith("RTL")), None)
    print(f"[timeout] phases: {' -> '.join(phases)}   RTL began at "
          f"{'%.0fs' % rtl_t if rtl_t is not None else 'never'}")
    esp_sim.save(data, args.out)
    # RTL should trigger from the time limit (~18 s in), then land.
    ok = (rtl_t is not None and rtl_t >= 15.0 and "LANDED" in phases)
    print("[timeout] " + ("PASS (max flight time -> RTL -> landed)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="max_timeout.json")
    ap.add_argument("--wait", type=float, default=180.0)
    sys.exit(run(ap.parse_args()))
