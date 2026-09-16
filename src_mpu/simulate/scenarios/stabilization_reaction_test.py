"""
Stabilization Reaction Test  (OPEN-LOOP)
----------------------------------------
Inject a fake tilt and confirm the firmware pushes the correct motors the
correct way. This exercises the attitude loop the other HIL scenarios never
touch (they leave the IMU at zero): the roll/pitch PIDs and the motor mixing
in MotorController.hpp.

WHAT THIS CATCHES: sign / axis / mixing / clamp bugs -- the class that flips a
drone in the first second (e.g. a flipped roll sign, or roll and pitch swapped).

WHAT THIS CANNOT CATCH (needs a real tethered flight): PID tuning, whether the
correction magnitude actually settles the drone, and whether a positive roll
NUMBER really means the drone is leaning right (that depends on IMU mounting +
the Madgwick filter, which only run on hardware). The injected tilt is a frozen
snapshot -- it does NOT react to the motors, so there is no feedback loop here.

Motor layout (see hardware/EspESC.hpp):
        FRONT
    M1 (front-left)   M2 (front-right)
    M3 (rear-left)    M4 (rear-right)
"""
import time

# No GPS fix on purpose: MANUAL doesn't need one to arm, and leaving fix=False
# disables MANUAL's GPS position-hold, so centered sticks give pure level
# targets (roll=0, pitch=0) -- any injected tilt is then a clean error.
set_world(alt_ft=10.0, heading_deg=90.0)  # fix stays False (the default)
time.sleep(0.5)  # let the barometer/heading loops publish before we enter

# Enter MANUAL from the ground, sticks centered -> level, heading-hold targets.
set_sticks(throttle=0.5, roll=0.0, pitch=0.0, yaw=0.0)
send("CRSFMANUAL:1")
assert approve_gate("MANUAL", reason="MANUAL_ON", timeout=5), "Did not enter MANUAL"

# Give the altitude loop a little headroom so base throttle > 0 (target 10 was
# captured on entry; drop the real altitude to 8 so there's a climb command).
set_world(alt_ft=8.0)
time.sleep(0.5)

# ---- LEVEL baseline: no tilt -> roll/pitch corrections ~0 -------------------
set_attitude(roll_deg=0.0, pitch_deg=0.0)
time.sleep(0.3)
level = query_motor()
assert level is not None, "No [MOTOR] response at level"
assert abs(level["roll"]) < 0.02 and abs(level["pitch"]) < 0.02, \
    f"Corrections should be ~0 when level, got roll={level['roll']} pitch={level['pitch']}"

# ---- ROLL RIGHT: right motors (M2,M4) must lift vs left (M1,M3) -------------
set_attitude(roll_deg=10.0, pitch_deg=0.0)
time.sleep(0.3)
r = query_motor()
assert r is not None, "No [MOTOR] response for roll-right"
assert r["roll"] < 0, f"Roll-right should give a negative roll correction, got {r['roll']}"
assert r["m2"] > r["m1"] and r["m4"] > r["m3"], \
    f"Roll-right should lift the RIGHT motors (m2>m1, m4>m3), got {r}"

# ---- ROLL LEFT: exact mirror -----------------------------------------------
set_attitude(roll_deg=-10.0, pitch_deg=0.0)
time.sleep(0.3)
rl = query_motor()
assert rl["roll"] > 0, f"Roll-left should give a positive roll correction, got {rl['roll']}"
assert rl["m1"] > rl["m2"] and rl["m3"] > rl["m4"], \
    f"Roll-left should lift the LEFT motors (m1>m2, m3>m4), got {rl}"

# ---- PITCH: front/rear balance must REVERSE between +pitch and -pitch -------
set_attitude(roll_deg=0.0, pitch_deg=10.0)
time.sleep(0.3)
p = query_motor()
assert p["pitch"] < 0, f"Pitch +10 should give a negative pitch correction, got {p['pitch']}"
front_p, rear_p = (p["m1"] + p["m2"]) / 2, (p["m3"] + p["m4"]) / 2

set_attitude(roll_deg=0.0, pitch_deg=-10.0)
time.sleep(0.3)
pn = query_motor()
assert pn["pitch"] > 0, f"Pitch -10 should give a positive pitch correction, got {pn['pitch']}"
front_n, rear_n = (pn["m1"] + pn["m2"]) / 2, (pn["m3"] + pn["m4"]) / 2

assert (front_p - rear_p) * (front_n - rear_n) < 0, \
    ("Pitch did not reverse the front/rear motor balance -- pitch axis may be "
     f"wrong: +pitch front={front_p:.3f} rear={rear_p:.3f}, "
     f"-pitch front={front_n:.3f} rear={rear_n:.3f}")

# ---- CLAMP: an extreme tilt must not push any motor outside 0..1 ------------
set_attitude(roll_deg=90.0, pitch_deg=0.0)
time.sleep(0.3)
c = query_motor()
for k in ("m1", "m2", "m3", "m4"):
    assert 0.0 <= c[k] <= 1.0, f"Motor {k} left the 0..1 range on an extreme tilt: {c[k]}"

# Level out before finishing.
set_attitude(0.0, 0.0)

forbid("[PANIC]")
