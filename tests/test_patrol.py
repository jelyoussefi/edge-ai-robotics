# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
import sys, numpy as np
sys.path.insert(0, "services/sim")
import os
os.environ["PATROL_LEG_M"]="1.0"; os.environ["PATROL_TURN_S"]="0.5"
import importlib, behaviours; importlib.reload(behaviours)
from behaviours import AvoidBehaviour, CRUISE_VX

b = AvoidBehaviour()
def obs(obstacles, t): b.observe({"obstacles": obstacles, "stamp": t})

# --- 1. Clear path: walks forward at cruise ---
obs([], 0.0)
c = b.command(0.0)  # first call, dt=0
c = b.command(0.1)  # dt=0.1
assert c[0] > 0, f"should walk forward, got {c}"
print(f"clear path -> vx={c[0]:.2f} (cruise, walking)  state={b._state}")

# --- 2. Walk a full leg (1.0m at 0.45 m/s ~ 2.2s), then it should TURN ---
t = 0.1
for _ in range(60):
    t += 0.1
    obs([], t)
    c = b.command(t)
    if b._state == "turn":
        break
print(f"after {b._leg_travelled:.2f}m -> state={b._state}  cmd={np.round(c,2)}")
assert b._state == "turn", "should switch to turn at leg end"
assert c[2] != 0 or np.allclose(c,0), "turning should yaw"

# --- 3. Complete the turn (0.5s), should return to WALK with reset leg ---
for _ in range(8):
    t += 0.1
    obs([], t)
    c = b.command(t)
    if b._state == "walk":
        break
print(f"after turn -> state={b._state}  leg reset to {b._leg_travelled:.2f}m")
assert b._state == "walk" and b._leg_travelled < 0.2, "should walk again, leg reset"

# --- 4. Obstacle close ahead: should brake (vx reduced) ---
b2 = AvoidBehaviour()
obs2 = lambda o,t: b2.observe({"obstacles": o, "stamp": t})
obs2([{"range_m": 0.6, "bearing_deg": 0.0}], 0.0)
b2.command(0.0); 
obs2([{"range_m": 0.6, "bearing_deg": 0.0}], 0.1)
c = b2.command(0.1)
print(f"obstacle 0.6m ahead -> vx={c[0]:.2f} (braked from {CRUISE_VX})")
assert c[0] < CRUISE_VX, "should brake for head-on obstacle"

# --- 5. Obstacle to the right: should steer left (positive wz) ---
b3 = AvoidBehaviour()
obs3 = lambda o,t: b3.observe({"obstacles": o, "stamp": t})
obs3([{"range_m": 0.8, "bearing_deg": 30.0}], 0.0)
b3.command(0.0)
obs3([{"range_m": 0.8, "bearing_deg": 30.0}], 0.1)
c = b3.command(0.1)
print(f"obstacle on right -> wz={c[2]:+.2f} (steers away)")
assert c[2] > 0, "obstacle on right should steer left"

print("\nPASS: patrol cycles walk<->turn, brakes and steers around obstacles")
