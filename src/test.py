import numpy as np
from rocket_landing.params import VehicleParams
from rocket_landing.integrators import euler_step

p = VehicleParams()
state = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, p.initial_mass])
control_zero_thrust = np.array([0.0, 0.0, 0.0])

dt = 0.1
for _ in range(10):
    state = euler_step(state, control_zero_thrust, p, dt)


print("z (height) after 1s:", state[2])
print("vz (vert. speed) after 1s:", state[5])


# Compare against the EXACT textbook formulas for constant-acceleration
# free fall (no integrator involved at all - pure algebra):
g = p.g
t = 1.0
print("exact z(1):", 1000.0 - 0.5 * g * t**2)
print ("exact vz(1):", -g * t)