import numpy as np
from rocket_landing.integrators import rk4_step


def simulate(state0, control, params, dt, max_steps=100000):
    """
    Runs a full simulation starting from 'state0', repeatedly
    calling rk4_step with a CONSTANT 'control' (same thrust/
    alpha/beta every step), and STOPS as soon as the rocke touches
    the ground ( z <= 0 ) - instead of running forever or crashing into
    negative altitude nonsensically.

    Returns:
        times   - 1D array, one timestamp per recorded state (starts at 0)
        stated  - 2D array, one row per recorded state (so states[-1] is
                  the final/landing state, stated[-1][2] is landing height)
    """
    times = [0.0]
    states = [np.array(state0, dtype=float)]

    for i in range(max_steps):
        current_state = states[-1] # latest state from states

        next_state = rk4_step(current_state, control, params, dt)
        times.append(times[-1] + dt)
        states.append(next_state) # adding new next steps to the current state

        #  z - height
        if next_state[2] <= 0.0:
            # Touched (or passed through) the ground - stop right there,
            # don't keep simulating a rocket that's already landed/crashed
            break

    return np.array(times), np.array(states)