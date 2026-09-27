#!/usr/bin/env python3
"""
ON-ESP scenario: RADIO LINK LOSS -> RTL.

Boot-selects the `rcloss` scenario: the sim fakes a working radio (the same
"last radio frame" timestamp the real receiver code sets), then cuts it 10 s
into the mission. With no radio the pilot can't reach the drone, so the radio
failsafe must bring it home (RTL) and land -- instead of finishing the mission.
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="rcloss", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[rcloss] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    print(f"[rcloss] phases: {' -> '.join(phases)}")
    esp_sim.save(data, args.out)
    # Came home and landed, and did NOT finish the mission normally (HOVER_SETTLE
    # only happens after the last waypoint).
    ok = (any(p.startswith("RTL") for p in phases) and "LANDED" in phases
          and "HOVER_SETTLE" not in phases)
    print("[rcloss] " + ("PASS (radio lost -> RTL -> landed)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="rc_loss.json")
    ap.add_argument("--wait", type=float, default=180.0)
    sys.exit(run(ap.parse_args()))
