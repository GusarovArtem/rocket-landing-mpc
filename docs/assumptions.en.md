# Physical model assumptions

This file describes the modeling decisions made at the start of the project,
following `thesis/plan_original.md` (section 2). Any change to these
assumptions during the project should be recorded here, so that the thesis's
methodology chapter stays consistent with what was actually implemented.

## Coordinate system

Local inertial frame, landing pad at the origin:

- `x` — east
- `y` — north
- `z` — vertically up
- gravity acts along the negative `z` axis

## State vector

```
state = [x, y, z, vx, vy, vz, m]
```

7 numbers: position (3), velocity (3), current mass (1).

## Control vector

```
control = [thrust, alpha, beta]
```

- `thrust` — engine thrust magnitude [N]
- `alpha`, `beta` — angles defining the thrust vector's direction relative to the `z` axis

Unit thrust-direction vector:

```
d(alpha, beta) = [sin(alpha) * cos(beta), sin(beta), cos(alpha) * cos(beta)]
```

(has length 1 for any `alpha`, `beta` — this is a mathematical property of
this parametrization, worth checking with a test).

## What IS INCLUDED in the first version of the model

- Gravity, constant, along `-z`.
- Variable mass (fuel consumption proportional to thrust, via specific impulse).
- Bounded thrust (`Tmin <= T <= Tmax`).
- Bounded tilt angle of the thrust vector from vertical.
- Three-dimensional motion of the center of mass.

## What IS EXPLICITLY EXCLUDED / SIMPLIFIED in the first version

- **No aerodynamic drag** — `F_D = 0`. Will be added later as an
  (optional) extension, if time allows.
- **No orientation dynamics (6-DoF)** — the rocket's body is conventionally
  treated as "fixed," not rotating; the thrust direction is controlled
  directly as a unit vector, without modeling steering engines, moments of
  inertia, or quaternions.
- **No propellant slosh dynamics inside the tanks, no body flexibility, no
  engine turbomachinery.**

These simplifications are deliberate and are described in
`thesis/plan_original.md` (section 1, "What does not need to be included in
the first version").

## Vehicle parameters

Initially, the educational rocket model's parameters will be plain fields in
a Python class (`src/rocket_landing/params.py`), not a YAML file — this is
simpler for someone who is learning Python. Migration to `configs/*.yaml` is
planned for Phase E (experiments), once easy scenario-switching is needed.
