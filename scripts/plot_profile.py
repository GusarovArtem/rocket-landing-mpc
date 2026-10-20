import numpy as np
import matplotlib.pyplot as plt

from rocket_landing.params import VehicleParams
from rocket_landing.actuators import clamp_control
from rocket_landing.simulation import simulate


def plot_fixed_thrust_profile():
    """
    Runs ONE simulation with a CONSTANT control (same thrust the whole
    time, straight up) and plots now height (z), vertical speed (vz) and
    mass (m) evolve over time, i three stacked subplots sharing the same
    time axis.
    """
    p = VehicleParams()
    control = clamp_control(np.array([200000.0, 0.0, 0.0]), p)

    # start at rest, 500m up, high enough to give a clearly visible
    # curve, low enough that the simulation finished in a handful
    # of seconds instead of minutes
    state0 = np.array([0.0, 0.0, 500.0, 0.0, 0.0, 0.0, p.initial_mass])

    times, states = simulate(state0, control, p, dt=0.05) # 50 ms


    height = states[:, 2] # select ALL z column values
    vertical_speed = states[:, 5]
    mass = states[:, 6]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(7, 8))

    axes[0].plot(times, height)
    axes[0].set_ylabel("height z [m]")
    axes[0].grid(True)

    axes[1].plot(times, vertical_speed)
    axes[1].set_ylabel("vertical speed vz [m/s]")
    axes[1].grid(True)

    axes[2].plot(times, mass)
    axes[2].set_ylabel("mass m [kg]")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(True)

    fig.suptitle("Fixed thrust = 200000 N, straight up, starting from rest at 500 m")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_fixed_thrust_profile()