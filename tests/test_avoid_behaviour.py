import sys, numpy as np
sys.path.insert(0, "services/sim")
from behaviours import AvoidBehaviour, CRUISE_VX, INFLUENCE_M, DANGER_M, PATROL_LEG_M, PATROL_TURN_S

b = AvoidBehaviour()
NOW = 0.0
def step(dt, obstacles=None):
    global NOW
    NOW += dt
    b.observe({"obstacles": obstacles or [], "stamp": NOW})
    return b.command(NOW)

# 1. Clear path -> cruises forward.
step(0.1, [])
c = step(0.1, [])
assert c[0] > 0 and abs(c[2]) < 1e-6, f"should cruise straight, got {c}"
print(f"clear path        -> vx={c[0]:.2f} wz={c[2]:+.2f}  cruises")

# 2. Walk a full leg -> should switch to TURN (about-face).
NOW = 0.0; b.__init__()
turned = False
for _ in range(400):
    c = step(0.1, [])
    if abs(c[2]) > 0.5 and c[0] < 0.05:   # turning in place
        turned = True; break
assert turned, "never reached about-face after a full leg"
print(f"end of {PATROL_LEG_M}m leg   -> turns in place (wz={c[2]:+.2f})  about-face")

# 3. Obstacle dead ahead (no depth, tall box) -> brakes and/or steers.
NOW = 0.0; b.__init__()
step(0.1, [])
c = step(0.1, [{"height": 0.7, "bearing_deg": 0.0}])   # big box straight ahead
assert c[0] < CRUISE_VX, f"should brake for head-on obstacle, got vx={c[0]:.2f}"
print(f"box ahead (h=0.7) -> vx={c[0]:.2f} < cruise {CRUISE_VX}  brakes")

# 4. Obstacle to the right -> steers left (positive wz).
NOW = 0.0; b.__init__()
step(0.1, [])
c = step(0.1, [{"height": 0.5, "bearing_deg": 30.0}])   # to the right
assert c[2] > 0, f"obstacle on right should steer left (+wz), got {c[2]:+.2f}"
print(f"box on right      -> wz={c[2]:+.2f} > 0  steers away (left)")

# 5. Mirror: obstacle to the left -> steers right.
NOW = 0.0; b.__init__()
step(0.1, [])
c = step(0.1, [{"height": 0.5, "bearing_deg": -30.0}])
assert c[2] < 0, f"obstacle on left should steer right (-wz), got {c[2]:+.2f}"
print(f"box on left       -> wz={c[2]:+.2f} < 0  steers away (right)")

print("\nPASS: patrols, about-faces, brakes head-on, steers around, symmetric")
