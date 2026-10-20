# Original thesis description (kept for reference)

> This file contains the full, original thesis description provided at the
> start of the project. It is kept here as a reference point when writing the
> thesis chapters (methodology, mathematical model, MPC, experiments). The
> actual execution sequence of the project — without a week-by-week split, as
> a flat list of steps — is in `PROGRESS.md` in the repository root.

## Thesis topic

Autonomous rocket landing and trajectory control

Developing a control algorithm for the rocket's trajectory during landing,
taking into account constraints on engine thrust, velocity, and position.

What I will learn: Model Predictive Control (MPC), trajectory optimization,
flight dynamics, and numerical simulation.

## 4-month thesis plan

### Autonomous rocket landing using MPC control

The goal is to create not just a rocket-landing animation, but an
engineering project from the field of Guidance, Navigation and Control
(GNC). The project includes a three-dimensional model of a returning
first-stage rocket, an autonomous MPC controller, thrust and dynamics
constraints, repeatable experiments, and a comparison against a baseline
controller.

This is a good direction for preparing to work in teams dealing with flight
control, navigation, trajectory optimization, and spacecraft landing. For
the project to be completable within four months, it is recommended to
first develop a simplified but physically consistent 3D model with a fixed
body orientation. Then, if time remains, orientation control can be added.

### Main task

3D modeling · MPC · Constraints · Numerical experiments

Developing a controller that guides the returning rocket stage toward the
landing site, reduces vertical and horizontal velocity before touchdown,
and respects the actuator system's constraints.

Duration: 16 weeks, i.e. four one-month stages.
Programming language: Python.
Libraries and tools: NumPy, SciPy, CasADi, IPOPT.
3D model: position, velocity, mass, and thrust.
Comparison: PID vs. MPC, plus robustness tests.

## 1. Defining the project scope

Proposed thesis title:

Autonomous Rocket Landing Using Model Predictive Control: Three-Dimensional
Trajectory Optimization Under State and Actuator Constraints

### Main research question

To what extent does MPC improve the accuracy and quality of a returning
rocket's autonomous landing compared to a baseline PID controller, under
limited thrust, model uncertainty, and the presence of disturbances?

### Project scope

- Three-dimensional translational dynamics of the rocket.
- Gravity, variable mass, and limited engine thrust.
- MPC controller with a finite prediction horizon and constraints.
- PID controller as a baseline algorithm for comparison.
- Modeling of wind, initial-velocity error, mass variations, and model
  errors.
- Landing metrics, failure analysis, and repeatable plots.

### What does not need to be included in the first version

One should not start right away with full modeling of aerodynamics, a
flexible body, turbomachinery, the engine, propellant fluid dynamics inside
the tanks, or complete six-degree-of-freedom (6-DoF) orientation control.
These are separate, complex topics.

In the first version, "3D model" means three-dimensional motion of the
center of mass. The rocket's body may be conventionally held vertical, and
the thrust direction may be controlled within an allowed tilt cone. This
makes it possible to study spatial landing without having to simultaneously
solve the full orientation problem.

Orientation and angular dynamics can be added later as a project extension.

## 2. Mathematical formulation

### 2.1. Coordinate system and rocket state

A local inertial coordinate system should be chosen:

- x axis — east;
- y axis — north;
- z axis — vertically upward;
- the landing site is at the origin of the coordinate system;
- gravity acts along the negative z axis.

Rocket state: `[x, y, z, vx, vy, vz, m]^T`, where `r` denotes position, `v`
— velocity, and `m` — the current mass.

Control: `u = [T, alpha, beta]^T`, where `T` denotes engine thrust, while
`alpha` and `beta` are parameters defining the thrust direction. They can be
used to define the unit vector of the engine's thrust direction:

```
d(alpha, beta) = [ sin(alpha)*cos(beta), sin(beta), cos(alpha)*cos(beta) ]
```

With this parametrization the vector has unit length. For small tilt
angles this is a convenient model. Before implementation, one should check
that the chosen angles correspond to the physical layout of the axes.

### 2.2. Equations of motion

In the first version, translational dynamics with variable mass should be
used:

```
dr/dt = v
dv/dt = (T/m) * d(alpha, beta) + g + F_D / m
dm/dt = -T / (Isp * g0)
```

where:

- `T` — engine thrust magnitude, in newtons;
- `Isp` — the engine's specific impulse, in seconds;
- `g0` — standard gravitational acceleration;
- `g` — gravitational acceleration vector;
- `F_D` — aerodynamic drag force.

In the first stage, zero aerodynamic drag can be assumed, but this
assumption must be clearly described in the thesis. In the second stage, a
simplified drag model can be added:

```
F_D = -0.5 * rho * C_D * A * |v_rel| * v_rel
```

where `rho` denotes air density, `C_D` — the drag coefficient, and `A` —
the effective area.

Important model limitation: the body does not rotate in these equations,
and the thrust-vector direction is controlled directly. This is therefore
a three-dimensional trajectory model with a steerable thrust vector, but
not a full 6-DoF rocket model.

A full model requires the use of quaternions or orientation matrices,
angular velocities, moments of inertia, and the dynamics of the actuators
that steer the thrust vector.

### 2.3. Constraints

The controller should determine not only a trajectory leading to the
landing site, but also a trajectory that is admissible from the point of
view of physical constraints.

Thrust constraints: `Tmin <= T_k <= Tmax`.

If the engine cannot be shut off during the flight phase under
consideration, the corresponding minimum-thrust constraint must be
included. If shutting off the engine is allowed, it must be modeled
separately — one must not automatically assume that the engine can
produce any thrust from zero up to the maximum.

Thrust-direction tilt constraint: `sqrt(dx^2 + dy^2) <= sin(theta_max)`,
where `theta_max` denotes the maximum tilt angle from vertical.

State constraints: `z_k >= 0`, `m_k >= m_dry`, where `m_dry` denotes the
rocket stage's dry mass without propellant.

Soft-landing terminal conditions: `|r_N - r_pad| <= epsilon_r`,
`|v_N| <= epsilon_v`. The altitude should be close to zero, and the
horizontal deviation must stay within the landing site's radius.

At the start, the parameters of an educational rocket model should be
chosen, not of a real rocket. For example, the initial mass, dry mass,
maximum thrust, and specific impulse should be defined in a configuration
file. Their values must be checked for physical plausibility. The model
should not require acceleration that the engine is unable to provide.

## 3. MPC task formulation

Model Predictive Control means that, at every step, the algorithm:

1. Receives the current state estimate.
2. Solves an optimization problem over a finite prediction horizon.
3. Applies only the first control from the computed sequence.
4. After the next step, measures the state again and computes the optimal
   control anew.

It is precisely this re-optimization that distinguishes MPC from a
trajectory computed in advance and executed without ongoing correction.

### 3.1. Objective function

Suppose the horizon spans `N` steps, and the duration of one step is `dt`.
The optimization problem has the form `min_u J`.

Example cost function:

```
J = sum_{k=0}^{N-1} [ ||r_k - r_ref,k||^2_Qr
                     + ||v_k - v_ref,k||^2_Qv
                     + ||u_k - u_ref,k||^2_R
                     + ||du_k||^2_S ]
  + ||r_N - r_pad||^2_Qf
  + ||v_N||^2_Pf
  + w_m * (m_0 - m_N)
```

Interpretation of the terms:

- `Qr` — weight penalizing deviation from the desired trajectory;
- `Qv` — weight penalizing excessive velocity;
- `R` — weight determining the significance of the control signal's
  magnitude;
- `S` — weight penalizing abrupt control changes;
- `Qf` and `Pf` — weights determining the importance of landing accuracy
  and low touchdown velocity;
- the last term encourages fuel savings by limiting mass consumption.

This is an example cost function, not the only correct solution. The
weights should be chosen experimentally and their values documented.

An excessive penalty on fuel consumption can lead to landing at high
velocity if the terminal conditions are defined solely through soft
penalties. Therefore, the landing task should use hard constraints or
verifiable terminal conditions, rather than relying solely on the cost
function's weights.

### 3.2. Discretization of the dynamics

The optimizer requires a discrete model: `x_{k+1} = f_d(x_k, u_k)`.

For the thesis, fourth-order Runge-Kutta (RK4) integration is recommended.
It typically provides greater accuracy than the simple explicit Euler
method for a comparable integration step.

An important engineering point: one should use the same physical equations,
but independent integrator implementations for the MPC prediction and for
the simulator. This avoids artificially perfect results arising from
identical numerical errors in both models.

### 3.3. First-prototype settings

| Parameter | Recommended initial value |
| --- | --- |
| Simulation step | 0.02-0.05 s |
| Control period | 0.1-0.2 s |
| Prediction horizon | 20-40 steps |
| Optimizer | CasADi + IPOPT |

These are initial values for performance profiling, not a guarantee of
convergence or real-time operation.

With a control period of 0.1 s and a horizon of 30 steps, the optimizer
predicts motion 3 seconds ahead. Later, an analysis of the effect of
horizon length, control frequency, and solve time should be carried out.

In the first implementation, it is worth using CasADi for the symbolic
description of the dynamics and for computing derivatives. If the
nonlinear optimization turns out to be too slow, one can consider a
simplified linear model, sequential quadratic programming, or a
specialized optimal-control problem formulation.

## 4. Python project architecture

The project should be split into independent modules. The dynamics should
not depend on a specific controller, and the controller should not depend
on the visualization method. This way, PID can be replaced by MPC without
rewriting the simulator.

```
rocket-landing-mpc/
├── pyproject.toml
├── README.md
├── configs/
│   ├── vehicle.yaml
│   ├── simulation.yaml
│   └── mpc.yaml
├── src/
│   └── rocket_landing/
│       ├── dynamics.py
│       ├── integrators.py
│       ├── environment.py
│       ├── actuators.py
│       ├── reference.py
│       ├── controllers/
│       │   ├── base.py
│       │   ├── pid.py
│       │   └── mpc.py
│       ├── simulation.py
│       ├── metrics.py
│       └── visualization.py
├── experiments/
│   ├── nominal.py
│   ├── robustness.py
│   ├── horizon_sweep.py
│   └── monte_carlo.py
├── tests/
│   ├── test_dynamics.py
│   ├── test_integrators.py
│   ├── test_constraints.py
│   └── test_metrics.py
├── results/
└── thesis/
```

### Responsibilities of individual modules

| Module | Scope of implementation |
| --- | --- |
| `dynamics.py` | Equations of motion, mass consumption, aerodynamics |
| `integrators.py` | RK4 and numerical state propagation |
| `environment.py` | Gravity, wind, atmosphere parameters |
| `actuators.py` | Thrust constraints, delays, control rate of change |
| `reference.py` | Reference trajectory or terminal goal |
| `controllers/pid.py` | Baseline controller |
| `controllers/mpc.py` | Optimization problem, constraints, warm-start initialization |
| `simulation.py` | Closed loop: state → control → motion |
| `metrics.py` | Landing errors, mass consumption, constraint violations |
| `visualization.py` | 3D trajectories and state/control plots |

### Recommended toolset

- **NumPy and SciPy** — numerical computation, integration, and helper
  algorithms.
- **CasADi + IPOPT** — building the nonlinear optimization problem and
  computing derivatives.
- **Matplotlib** — plots of trajectory, velocity, thrust, mass, and
  controller comparisons.
- **pytest** — automated tests of the physical model, constraints, and
  metrics.

At the start, it is not worth spending time on complicated 3D graphics. A
3D plot in Matplotlib together with separate time-series plots is entirely
sufficient for the thesis.

### Example dynamics-model interface

```python
import numpy as np


def dynamics(state, control, params, wind=None):
    # state = [x, y, z, vx, vy, vz, mass]
    # control = [thrust, alpha, beta]

    position = state[:3]
    velocity = state[3:6]
    mass = state[6]

    thrust, alpha, beta = control

    direction = np.array([
        np.sin(alpha) * np.cos(beta),
        np.sin(beta),
        np.cos(alpha) * np.cos(beta),
    ])

    gravity = np.array([0.0, 0.0, -params.g])
    acceleration = thrust / mass * direction + gravity

    # Air drag will be added in a separate module.
    mass_rate = -thrust / (params.isp * params.g0)

    return np.concatenate([
        velocity,
        acceleration,
        [mass_rate],
    ])
```

This is only an initial version of the function. It does not yet account
for air drag, admissible-state checks, an actuator model, or constraints.
Before using it in MPC, tests for dimensions, gravity direction, mass
conservation, and the correctness of the unit thrust vector should be
added.

## 5. Detailed 16-week plan (original, replaced by the flat list of steps)

> Note: this weekly schedule is kept only as a reference point. The actual
> execution of the project follows the flat list of steps in `PROGRESS.md`,
> without rigid weekly deadlines — the user works daily, at their own pace,
> until all steps are completed.

### Month 1 — dynamics and physical model

Outcome: a verified 3D simulator without MPC.

- Week 1: learning the basic concepts related to control, defining
  assumptions, defining the coordinate system and rocket parameters.
- Week 2: implementing the equations of motion and the RK4 integrator;
  writing tests for gravity, thrust, and mass consumption.
- Week 3: adding the landing site, initial conditions, thrust constraints,
  and ground-contact detection.
- Week 4: creating a three-dimensional trajectory and velocity/mass plots;
  manually checking a few scenarios.

Checkpoint: the model moves predictably under a given control, and the
tests pass correctly.

### Month 2 — baseline controller and MPC

Outcome: two working algorithms.

- Week 5: implementing PID for horizontal-deviation and vertical-velocity
  control. Tuning parameters for the nominal scenario.
- Week 6: learning the MPC formulation; implementing the discrete model
  and the cost function.
- Week 7: building the optimization problem in CasADi and adding state and
  control constraints.
- Week 8: implementing receding-horizon control, warm-start, and handling
  of optimizer solve failures.

Checkpoint: MPC can carry out a nominal landing in simulation, and the
causes of non-convergence are logged.

### Month 3 — experiments and robustness

Outcome: an objective comparison of the algorithms.

- Week 9: standardizing initial conditions and metrics for PID and MPC.
- Week 10: adding wind, initial-velocity estimation error, and a mismatch
  between the controller's model and the simulator's model.
- Week 11: running a series of tests with different horizons, weights, and
  constraints.
- Week 12: running Monte Carlo experiments and preparing tables and plots.

Checkpoint: a repeatable set of experiments exists, accounting for failed
landings and non-convergence cases.

### Month 4 — analysis, thesis writing, and defense

Outcome: a completed scientific-technical thesis.

- Week 13: verifying numerical accuracy, constraint correctness, and
  result sensitivity.
- Week 14: describing the methodology, the mathematical model, MPC, and
  the experiment configuration.
- Week 15: writing up the results, conclusions, and model limitations, as
  well as the README file and run instructions.
- Week 16: preparing the presentation, demo, and answers to committee
  questions.

Checkpoint: every main result presented in the thesis can be reproduced
from the code and the documented settings.

## 6. Experiment plan

Experiments should answer the research questions, not merely serve to
display attractive trajectories.

### Experiment A — nominal landing

Run PID and MPC with identical rocket parameters, initial conditions, and
constraints. Compare: horizontal deviation from the landing-site center,
vertical and total velocity at touchdown, fuel consumption, maximum thrust
deviation, number of constraint violations, control-computation time.

### Experiment B — varying the initial state

Vary the initial position and velocity within a set range. Check for which
initial states the algorithm can still ensure a soft landing.

### Experiment C — disturbances and model errors

Introduce, one at a time and then in combination: crosswind, initial-
velocity error, initial-mass error, a mismatch between actual and expected
thrust, a change in specific impulse, control delay, or a limited rate of
thrust change. It is especially important to separate the model used by
MPC from the simulator's actual model.

### Experiment D — MPC parameter tuning

Vary, one at a time: the prediction-horizon length, the terminal-error
weight, the control-change penalty, the control-recomputation frequency,
the admissible thrust-vector deviation. Measure landing quality and
computation time.

### Experiment E — Monte Carlo simulations

Randomly generate initial states and model parameters within previously
set ranges. For the first series, it is worth performing 100-300 runs,
recording the random-number-generator seed.

## 7. Metrics and success criteria

| Metric | Measurement method |
| --- | --- |
| Landing error | Horizontal distance of the touchdown point from the landing-site center |
| Touchdown velocity | Vertical and total velocity |
| Success rate | Fraction of scenarios meeting all soft-landing conditions |
| Fuel consumption | Mass change during the flight |
| Constraint violations | Number and magnitude of violations of thrust, mass, altitude, and angle constraints |
| Computation time | Median, 95th percentile, and maximum optimization time |
| Robustness | Change in success rate in the presence of disturbances |

Proposed educational criteria: physical correctness, nominal landing,
PID-vs-MPC comparison, robustness, repeatability. As preliminary numerical
values, a horizontal landing error of no more than 2 m and a touchdown
velocity of no more than 1 m/s can be adopted — to be verified as to
whether they are achievable with the chosen thrust and initial conditions.

## 8. Literature and NASA materials

- G-FOLD — Fuel Optimal Large Divert Guidance (NASA TechPort / NTRS).
- Entry, Descent & Landing Subsystems (NASA Johnson Space Center).
- Guidance, Navigation & Control Subsystems (NASA Johnson Space Center).
- NESC Academy — Guidance, Navigation, and Control (NASA training
  materials).
- JPL/Masten: precision landing guidance (NASA / Jet Propulsion
  Laboratory).
- Ping Lu, *Propellant-Optimal Powered Descent Guidance*, Journal of
  Guidance, Control, and Dynamics.
- Description of the SpaceX Starfall test stand (as professional context,
  not an academic requirement).

## 9. How to turn the thesis into a strong portfolio project

- Public code repository: clear structure, automated tests, pinned
  dependencies, configuration files, and instructions for reproducing the
  experiments.
- Short technical video: three-dimensional landing trajectory, velocity
  and thrust plots, PID-vs-MPC comparison, and an example scenario where
  one of the controllers fails.
- Short technical report in English: problem, model, constraints,
  algorithm, results, computational complexity, and limitations of the
  adopted assumptions.

## 10. What to do in the first seven days (original checklist)

- Create the repository and the Python virtual environment.
- Write down the model assumptions and the coordinate-system definitions.
- Choose plausible parameters for the educational rocket model and record
  them in the configuration.
- Implement the state `(x, y, z, vx, vy, vz, m)` and the equations of
  motion.
- Write a free-fall test with no thrust.
- Write a vertical-flight test with constant thrust.
- Generate the first plot of altitude and velocity as functions of time.

Main rule: first develop a credible physical model, then get a working
MPC running, and only afterward prove its advantage through experiments.
