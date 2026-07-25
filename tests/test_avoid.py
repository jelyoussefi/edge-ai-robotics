"""Obstacle avoidance: cruise, steer away, brake head-on, saturate, staleness."""
import sys, math, numpy as np
sys.path.insert(0, "services/sim")
from behaviours import AvoidBehaviour, INFLUENCE_M, DANGER_M, CRUISE_VX

b = AvoidBehaviour(); NOW = 1000.0
def obs(items): b.observe({"obstacles": items, "stamp": NOW})
def o(bearing, rng): return {"bearing_deg": bearing, "range_m": rng, "camera": 0}

# 1. Clear path -> cruise straight.
obs([]); c = b.command(NOW)
assert c[0] == CRUISE_VX and c[2] == 0.0, c
print(f"clear path           -> vx={c[0]:.2f} wz={c[2]:+.2f}   cruises   ok")

# 2. Obstacle far beyond influence -> ignored, still cruising.
obs([o(0, INFLUENCE_M + 1.0)]); c = b.command(NOW)
assert c[0] == CRUISE_VX and c[2] == 0.0
print(f"obstacle out of range-> cruises unchanged        ok")

# 3. Obstacle on the RIGHT -> robot steers LEFT (+wz).
obs([o(+30, 1.2)]); c = b.command(NOW)
assert c[2] > 0, f"right obstacle should steer left, wz={c[2]}"
print(f"obstacle right       -> wz={c[2]:+.2f} (steers left)   ok")

# 4. Mirror: obstacle LEFT -> steers RIGHT, symmetric.
obs([o(-30, 1.2)]); c2 = b.command(NOW)
assert math.isclose(c2[2], -c[2], rel_tol=1e-6), "left/right must mirror"
print(f"obstacle left        -> wz={c2[2]:+.2f} (symmetric)     ok")

# 5. Dead ahead and close -> forward speed cut (cannot steer around a head-on).
obs([o(0, DANGER_M)]); c = b.command(NOW)
assert c[0] < CRUISE_VX, f"head-on should brake, vx={c[0]}"
print(f"head-on at {DANGER_M}m     -> vx={c[0]:.2f} (brakes)        ok")

# 6. Closer pushes harder: nearer obstacle yields >= steering than farther.
obs([o(30, 2.0)]); far = abs(b.command(NOW)[2])
obs([o(30, 0.8)]); near = abs(b.command(NOW)[2])
assert near >= far, f"closer should push harder: near={near} far={far}"
print(f"closer pushes harder -> {far:.2f} -> {near:.2f}          ok")

# 7. Two symmetric obstacles -> lateral pushes cancel, robot brakes and goes straight.
obs([o(+25, 1.0), o(-25, 1.0)]); c = b.command(NOW)
assert abs(c[2]) < 1e-6, f"symmetric should cancel steer, wz={c[2]}"
assert c[0] < CRUISE_VX, "symmetric wall should still brake"
print(f"symmetric pair       -> wz={c[2]:+.2f} vx={c[0]:.2f} (brakes)  ok  [local min, expected]")

# 8. Unknown range (None) -> treated as far but noted, mild influence at worst.
obs([{"bearing_deg": 20, "range_m": None, "camera": 0}]); c = b.command(NOW)
assert math.isfinite(c[0])  # no crash, produces a command
print(f"unknown range        -> vx={c[0]:.2f} wz={c[2]:+.2f}   handled   ok")

# 9. Stale -> stop.
obs([o(0, 0.5)]); c = b.command(NOW + 5.0)
assert np.allclose(c, 0), f"stale should stop, got {c}"
assert b.status() == {"avoiding": False}
print(f"stale >0.7s          -> stops                     ok")

print("\nPASS")
