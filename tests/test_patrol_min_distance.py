import sys, numpy as np
sys.path.insert(0, "services/sim")
import os
os.environ["PATROL_MIN_DISTANCE"] = "2.7"
os.environ["PATROL_LEG_M"] = "2.5"
os.environ["PATROL_TURN_S"] = "1.0"
# reimport with env
import importlib, behaviours
importlib.reload(behaviours)
from behaviours import AvoidBehaviour, PATROL_MIN_DISTANCE, PATROL_LEG_M

b = AvoidBehaviour()
NOW = 0.0
def step(dt):
    global NOW
    NOW += dt
    b.observe({"obstacles": [], "stamp": NOW})
    return b.command(NOW)

# Simulate a long patrol, clear path, track camera distance.
min_seen = float("inf"); max_seen = 0.0
dists = []
for _ in range(2000):
    step(0.05)
    d = b.status()["camera_dist_m"]
    min_seen = min(min_seen, d)
    max_seen = max(max_seen, d)
    dists.append(d)

print(f"PATROL_MIN_DISTANCE = {PATROL_MIN_DISTANCE}")
print(f"PATROL_LEG_M        = {PATROL_LEG_M}")
print(f"distance caméra min observée : {min_seen:.2f} m")
print(f"distance caméra max observée : {max_seen:.2f} m")
print(f"plage attendue : [{PATROL_MIN_DISTANCE:.1f}, {PATROL_MIN_DISTANCE+PATROL_LEG_M:.1f}]")

# The robot must never go below PATROL_MIN_DISTANCE (small tolerance for one step overshoot).
tol = 0.2
assert min_seen >= PATROL_MIN_DISTANCE - tol, f"robot entered dead zone: {min_seen:.2f} < {PATROL_MIN_DISTANCE}"
# It should patrol outward, reaching near the far edge.
assert max_seen >= PATROL_MIN_DISTANCE + PATROL_LEG_M - 0.5, f"robot never patrolled far enough: {max_seen:.2f}"
# It should oscillate (come back), not just walk away forever.
assert min_seen <= PATROL_MIN_DISTANCE + 0.5, f"robot never returned toward camera: min {min_seen:.2f}"
print("\nPASS: patrouille bornée, ne rentre jamais dans la zone morte, oscille bien")
