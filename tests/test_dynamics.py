import numpy as np
import pytest
from scipy.differentiate import derivative

from rocket_landing.params import VehicleParams
from rocket_landing.dynamics import dynamics, free_fall_dynamics


def test_free_fall_acceleration():
    """
    In the free fall (no thrust), the only acceleration should be gravity,
    pointing straight down: [0, 0, -g].
    """
    p = VehicleParams()
    state = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, p.initial_mass ])

    derivative = free_fall_dynamics(state, p)

    # derivative = [vx, vy, vz, ax, ay, az, mass_rate] - we only care about
    # the acceleration part here, i.e indies 3, 4, 5
    acceleration = derivative[3:6]

    assert acceleration == pytest.approx([0.0, 0.0, -p.g])

def test_mass_conserved_at_zero_thrust():
    """
    If thrust is 0, the engine isn't burning any fuel, so mass not change at all
    d(mass)/dt should be exactly 0 (not even a tiny nonzero number from a bug),
    regardless of gravity or relocity.
    """
    p = VehicleParams()
    state = np.array([0.0, 0.0, 1000.0, 1.0, 2.0, -3.0, p.initial_mass])
    control = np.array([0.0, 0.0, 0.0]) # thrust=0, alpha=0, beta=0

    derivative = dynamics(state, control, p)

    # derivative[6] is d(mass)/dt - the last element, see dynamics()
    mass_rate = derivative[6]

    assert mass_rate == 0.0