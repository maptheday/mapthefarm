#!/usr/bin/env python3
"""
ON-ESP scenario: PERMANENT GPS LOSS -> LAND.

Boot-selects the `gpsloss` scenario, which drops the GPS fix once the drone is
hovering. With no GPS you can't fly home, so the failsafe must abort straight to
LANDING (never RTL) and touch down. (Replaces the old edge_gps_permanent_loss.)
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="gpsloss", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[gpsloss] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    print(f"[gpsloss] phases: {' -> '.join(phases)}")
    esp_sim.save(data, args.out)
    # GPS loss must go straight to LANDING (no RTL, which needs GPS) and land.
    ok = ("LANDING" in phases and "LANDED" in phases
          and not any(p.startswith("RTL") for p in phases))
    print("[gpsloss] " + ("PASS (GPS lost -> direct LANDING)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="gps_loss.json")
    ap.add_argument("--wait", type=float, default=120.0)
    sys.exit(run(ap.parse_args()))
