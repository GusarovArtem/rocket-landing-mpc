import numpy as np
import pytest

from rocket_landing.params import VehicleParams
from rocket_landing.integrators import euler_step, rk4_step



def test_rk4_is_more_accurate_that_euler_on_free_fall():
    """
    Sims 1 sec of free all with BOTH integrators (10 steps of dt=0.1
    each, same as the manual console checks from steps 14 and 17),
    then compare each one's final height against the EXACT textbook
    formula, RK4`s error should be dramatically smaller than Euler's
    """
    p = VehicleParams()
    dt = 0.1
    n_steps = 10

    state_euler = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, p.initial_mass])
    state_rk4 = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, p.initial_mass])
    control = np.array([0.0, 0.0, 0.0]) # pure free fall

    for _ in range(n_steps):
        state_euler = euler_step(state_euler, control, p, dt) #1 sec of free fall
        state_rk4 = rk4_step(state_rk4, control, p, dt) #1 sec of free fall

    t = n_steps * dt # 1.0 second total
    exact_z = 1000.0 - 0.5 * p.g * t**2

    euler_error = abs(state_euler[2] - exact_z)
    rk4_error = abs(state_rk4[2] - exact_z)

    # RK4 should match the exact answer almost perfectly
    assert rk4_error < 1e-6

    # Euler should be anywhere near that accurate
    assert euler_error > 0.1

    # The actual point of this test: RK4`s error must be 1000 times smaller
    # than Euler's - many orders of magnitude, not just "a bit better"
    assert rk4_error < euler_error / 1000
