import sys, numpy as np
sys.path.insert(0, "services/sim")
from behaviours import ReactiveBehaviour, COMFORT_M, PANIC_M

b = ReactiveBehaviour(); NOW = 1000.0

def obs(people): b.observe({"people": people, "stamp": NOW})
def person(cx, r): return {"cx": cx, "cy": 0.5, "range_m": r, "camera": 0, "score": 0.9}

# 1. Nobody -> stand still.
obs([]); assert np.allclose(b.command(NOW), 0), "should idle with no people"
print("no people          -> [0 0 0]                     ok")

# 2. Dead ahead, comfortable distance -> hold.
obs([person(0.5, 3.0)]); c = b.command(NOW)
assert np.allclose(c, 0), c
print("ahead, far         -> hold                        ok")

# 3. Person to the RIGHT of frame -> yaw NEGATIVE (turn right toward them).
obs([person(0.85, 3.0)]); c = b.command(NOW)
assert c[2] < 0, f"expected negative yaw, got {c[2]}"
print(f"right of frame     -> wz={c[2]:+.2f} (turns right)      ok")

# 4. Mirror: left of frame -> positive yaw, same magnitude.
obs([person(0.15, 3.0)]); c2 = b.command(NOW)
assert np.isclose(c2[2], -c[2]), "left/right must be symmetric"
print(f"left of frame      -> wz={c2[2]:+.2f} (symmetric)       ok")

# 5. Inside comfort radius -> back away.
obs([person(0.5, 1.0)]); c = b.command(NOW)
assert c[0] < 0, f"expected retreat, got vx={c[0]}"
print(f"inside {COMFORT_M}m       -> vx={c[0]:+.2f} (retreats)       ok")

# 6. Retreat saturates at panic distance, never exceeds it.
speeds = [b.command(NOW)[0] for r in (PANIC_M*0.5, PANIC_M, 1.0, COMFORT_M)
          for _ in [obs([person(0.5, r)])]]
assert speeds[0] == speeds[1], "should saturate below panic radius"
assert abs(speeds[0]) >= abs(speeds[2]) >= abs(speeds[3]), "must be monotonic"
print(f"retreat profile    -> {[round(s,2) for s in speeds]}  saturates ok")

# 7. Deadband: tiny offset must not twitch.
obs([person(0.52, 3.0)]); assert b.command(NOW)[2] == 0.0
print("small offset       -> deadband, no twitch         ok")

# 8. Stale detections -> stop, do not chase a ghost.
obs([person(0.9, 0.5)]); assert np.allclose(b.command(NOW + 5.0), 0)
assert b.status() == {"tracking": False}
print("stale >1s          -> stops, tracking=False       ok")

print("\nPASS")
