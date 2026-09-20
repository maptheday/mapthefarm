#!/usr/bin/env python3
"""
ON-ESP scenario: STABILIZATION RECOVERY.

Boot-selects the `stab` scenario: once hovering, a one-shot disturbance rolls the
drone ~22 deg (a gust). Because the physics is closed-loop on the chip, this tests
the REAL stabilization loop -- expect a big roll spike followed by the controller
bringing it back to level. (Replaces the old open-loop stabilization_reaction_test,
which could only check the correction's sign; this checks it actually recovers.)
"""
import argparse, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="stab", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[stab] NO SAMPLES -- is the `sim` build flashed?"); return 1
    hold = [x for x in s if x["phase"] == "HOLD"]
    peak = max((abs(x["roll"]) for x in hold), default=0.0)
    settled = max((abs(x["roll"]) for x in hold[-10:]), default=99.0)  # last ~1 s
    print(f"[stab] peak roll after kick: {peak:.1f} deg;  settled to {settled:.1f} deg")
    esp_sim.save(data, args.out)
    ok = (peak > 15.0 and settled < 5.0)   # got kicked, then recovered to level
    print("[stab] " + ("PASS (kicked ~22 deg, recovered to level)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="stabilization.json")
    ap.add_argument("--wait", type=float, default=60.0)
    sys.exit(run(ap.parse_args()))
