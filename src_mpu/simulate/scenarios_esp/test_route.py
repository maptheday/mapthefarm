#!/usr/bin/env python3
"""
ON-ESP scenario: THE SHORT FIRST-FLIGHT ROUTE.

Boot-selects the `testroute` scenario, which swaps the mission to TEST_ROUTE
(30 m out along the first fence leg and straight back, at 20 ft) -- the route
the real drone flies until USE_TEST_ROUTE is turned off. Expect: the mission
completes, it never strays far, and it lands back near the launch point.
"""
import argparse, math, sys
import esp_sim


def run(args):
    data = esp_sim.fly_and_pull(args.port, scenario="testroute", wait=args.wait)
    s = data["samples"]
    if not s:
        print("[testroute] NO SAMPLES -- is the `sim` build flashed?"); return 1
    phases = list(dict.fromkeys(x["phase"] for x in s))
    farthest = max((x["dist"] for x in s), default=0.0)
    home_dist = math.hypot(s[-1]["north"], s[-1]["east"])
    print(f"[testroute] phases: {' -> '.join(phases)}")
    print(f"[testroute] farthest {farthest:.0f} m from home, landed {home_dist:.1f} m from home")
    esp_sim.save(data, args.out)
    # Finished the route (HOVER_SETTLE only follows the last waypoint), stayed
    # close (the far point is 30 m out), and landed back near the launch point.
    ok = ("MISSION" in phases and "HOVER_SETTLE" in phases and "LANDED" in phases
          and farthest < 45.0 and home_dist < 10.0)
    print("[testroute] " + ("PASS (short route flown, landed at home)" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--out", default="test_route.json")
    ap.add_argument("--wait", type=float, default=180.0)
    sys.exit(run(ap.parse_args()))
