#!/usr/bin/env python3
"""
ON-ESP scenario: GEOFENCE BREACH -> RTL.

Boot-selects the `geofence` scenario, which shrinks the geofence so the drone
crosses it soon after takeoff. Expect the geofence failsafe to force Return-To-
Launch and then land back home. (Replaces the old edge_geofence_breach HIL test.)
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="geofence", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[geofence] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    print(f"[geofence] phases: {' -> '.join(phases)}")
    esp_sim.save(data, args.out)
    ok = any(p.startswith("RTL") for p in phases) and "LANDED" in phases
    print("[geofence] " + ("PASS (geofence -> RTL -> landed)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="geofence_breach.json")
    ap.add_argument("--wait", type=float, default=180.0)
    sys.exit(run(ap.parse_args()))
