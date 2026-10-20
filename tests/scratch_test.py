import numpy as np

from rocket_landing.params import VehicleParams
from rocket_landing.simulation import simulate

p = VehicleParams()
# Start LOW on purpose (50m), free fall (thrust=0), so it lands
# quickly and we can check the timing against a simple hand formula.
state0 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0, p.initial_mass])
control = np.array([0.0, 0.0, 0.0])

times, states = simulate(state0, control, p, dt=0.01) # 10 ms

print("number of recorded states:", len(states))
print("final time:", times[-1])
print("final z (should be <= 0):", states[-1][2]) # latest record z should be <= 0


# Hand calc: time to fall 50m under gravity alone
# from the textbook formula h = 0.5 * g * t^2 => t = sqrt(2*h/g)
exact_fall_time=np.sqrt(2 * 50.0 / p.g)
print("exact fall time:", exact_fall_time)