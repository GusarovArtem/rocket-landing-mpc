import numpy as np
from scipy.differentiate import derivative

from rocket_landing.dynamics import dynamics


def euler_step(state, control, params, dt):
    """
    Advances `state` forward by one small time step `dt`, using the
    SIMPLEST possible method: Euler integration.


    The idea: we already know the INSTANTANEOUS rate of change of the
    state (that's exactly what dynamics() computes - see its docstring).
    Euler integration just assumes that rate of change stays constant for
    the whole tiny duration `dt`, and takes one straight step using it:

        state(t + dt) ~= state(t) + dt * d(state)/dt


    This is an APPROXIMATION - it's only exactly correct if the rate of
    change truly never changes during the step (which is never quite
    true, except for velocity under exactly constant acceleration - see
    the check below). The smaller `dt` is, the smaller the error, but it
    never goes away completely. That's WHY we'll add a better integrator
    (RK4) later in step 17 - this EULER one is just a fast, simole
    starting point to get the simulator running end-to-end.
    """
    derivative = dynamics(state, control, params)
    return state + dt * derivative