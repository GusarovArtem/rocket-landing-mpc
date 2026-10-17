import numpy as np
from rocket_landing.params import VehicleParams
from rocket_landing.dynamics import dynamics

p = VehicleParams()
state = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, p.initial_mass])

# Case 1: zero thrust -> should be IDENTICAL to free_fall_dynamics.
control_zero_thrsut = np.array([0.0,0.0,0.0])
print(dynamics(state, control_zero_thrsut, p))

# Case 2: HOVER TEST - thrust exactly cancels gravity, straight up
# (alpha = beta = 0). weight = mass * g is the force needed to exactly
# cancel gravity at this instant.
weight = p.initial_mass * p.g
control_hover = np.array([weight, 0.0, 0.0])
print(dynamics(state, control_hover, p))