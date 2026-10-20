# PROGRESS — flat list of steps (no weeks)

> English translation of `PROGRESS.md`, kept as a snapshot for portfolio/
> reference purposes. **`PROGRESS.md` (Polish) is the live, authoritative
> file** that gets updated every session — this file is NOT automatically
> kept in sync with it, so re-translate it before relying on it for
> anything current.
>
> This is our cross-session project memory. Every session starts by
> checking this file: what's done, what's the "CURRENT STEP". Full task
> description: `thesis/plan_original.md` (English: `plan_original.en.md`).
> Model assumptions: `docs/assumptions.md` (English: `assumptions.en.md`).
>
> Mechanics: configuration/documentation steps are done directly by
> Claude. Steps involving physics/controller/MPC code go into
> `CODE_TO_BE_REWRITTEN.py` (a scratchpad file in the repo root, in
> `.gitignore`) — it contains code with English comments, to be manually
> transcribed into the real project file. After transcribing and running
> it, you report the result, and the step gets checked off, with the next
> one announced.

**CURRENT STEP: 24**

## Phase A — Setup and housekeeping

- [x] 1. Cleanup of old files: `main.py` removed (PyCharm boilerplate), the
      old 1D tutorial archived to `thesis/archive_pycharm_1d_tutorial.md`,
      `README.md` replaced with a short project overview.
- [x] 2. Original thesis description saved to `thesis/plan_original.md`.
- [x] 3. `pyproject.toml` filled in (metadata + dependencies: numpy, scipy,
      matplotlib, casadi, pytest as a dev extra).
- [x] 4. Installation smoke test: `import numpy, matplotlib, pytest` works
      (numpy 2.5.4, matplotlib 3.11.2, pytest 9.1.1, Python 3.14.8).
- [x] 5. CasADi installed (3.8.1) and tested: toy optimization problem
      (`min (x-1)^2+(y-2)^2`) solved by IPOPT, result `x=1, y=2` ✓.
      No installation problems on this machine (macOS, Python 3.14, arm64).
- [x] 6. Package skeleton: `src/rocket_landing/__init__.py` created.
- [x] 7. `PROGRESS.md` and `CODE_TO_BE_REWRITTEN.py` created, `.gitignore`
      updated (ignores the scratchpad + standard Python junk).
- [x] 8. Model assumptions saved to `docs/assumptions.md`.

*Phase A checkpoint: environment ready, no surprises installing
CasADi/IPOPT, directory structure in place.* ✅ PASSED

## Phase B — Physics and simulator (no controller)

- [x] 9. Rocket parameters class (`src/rocket_landing/params.py`): mass,
      dry mass, max thrust, Isp, g0, max tilt angle — plain Python
      fields. Checked in the Python console: `print(p.initial_mass,
      p.dry_mass, p.max_thrust, p.max_tilt_angle)` → `25000.0 22000.0
      400000.0 0.3490658503988659` ✓.
- [x] 10. Gravity / free-fall function (simplest possible `dynamics`).
      Verified in the console; fixed a transcription bug (acceleration
      had 2 elements instead of 3, due to the typo `0.0 - params.g`
      instead of `0.0, 0.0 - params.g`).
- [x] 11. Thrust-direction unit vector function `d(alpha, beta)` + a
      manual check that its length = 1. Verified in the console:
      `d(0,0)=[0,0,1]`, `d(max_tilt,0)=[0.342, 0, 0.9397]`,
      `norm(d(0.4,-0.25))=1.0` ✓.
- [x] 12. Full `dynamics()` function: gravity + thrust + mass consumption
      (no air drag). Verified in the console: zero-thrust == free-fall,
      hover test: acceleration ~0, mass_rate = -83.33 kg/s ✓.
- [x] 13. Euler integrator (the simplest one, for quick verification).
      Verified in the console: free fall, 10 steps at dt=0.1s.
- [x] 14. Manual script: free fall, printing height — compared against a
      hand calculation. vz matched perfectly (-9.80665), z was
      overestimated by ~0.49 m (the known, expected error of the Euler
      method) ✓.
- [x] 15. Manual script: vertical flight at constant thrust — compared
      against a hand calculation. Euler matched the hand calculation (z,
      vz, m) up to float rounding; m matched perfectly (no Euler error
      for mass, as expected) ✓.
- [x] 16. First pytest tests (`test_dynamics.py`): acceleration in free
      fall, mass conservation at zero thrust. 2 passed ✓. Fixed the
      second test: `== 0.0` instead of `pytest.approx(0.0)` (approx would
      have masked a real bug, since 0.0/x is always exactly zero).
- [x] 17. RK4 integrator (`integrators.py`), separate from Euler. Verified
      in the console: free fall for 1s, z and vz matched the exact
      formula PRACTICALLY perfectly (unlike the Euler error of ~0.49 m).
      Fixed a transcription bug (`k1 + 2 + k2` instead of `k1 + 2*k2`).
- [x] 18. Test comparing RK4 vs Euler on free fall (accuracy). 1 passed —
      RK4's error is over 1000x smaller than Euler's error ✓.
- [x] 19. Ground-contact detection (`z <= 0` → stop the simulation).
      Verified in the console: free fall from 50 m, dt=0.01. 321
      recorded states, final time ≈ 3.20 s (consistent with the exact
      fall time of 3.1933 s plus one "overshoot" step through the
      ground), final z ≈ -0.21 m (a small negative overshoot, expected
      at this dt) ✓.
- [x] 20. Thrust and tilt-angle constraints (`actuators.py`). Verified in
      the console: a wild control (999999, 5, -5) got clamped to
      (400000, 0.349.., -0.349..), a legal control (200000, 0.1, -0.1)
      came back UNCHANGED ✓.
- [x] 21. First plots: height/velocity/mass over time for a fixed thrust
      profile. Verified visually (`scripts/plot_profile.py`): height
      smoothly drops 500→0 in ~25 s, vz keeps getting more negative
      (never slows down — this thrust is too weak, as predicted), mass
      is perfectly linear 25000→~23300 kg ✓.
- [x] 22. First 3D trajectory plot (matplotlib) for a manually chosen
      thrust profile with a lateral component. Verified visually
      (`plot_3d_trajectory()`): trajectory from (0,0,800) to landing with
      clear drift in x and y, as predicted (constant thrust direction =>
      the trajectory is almost perfectly straight in space, only the
      magnitude changes over time, not the direction) ✓.

*Phase B checkpoint: the physical simulator works and is tested, with no
controller at all.* ✅ PASSED

## Phase C — Baseline PID controller

- [x] 23. Defining the goal (landing site, zero velocity) in
      `reference.py`. Verified in the console: `target.position =
      [0,0,0]`, `target.velocity = [0,0,0]` ✓.
- [ ] 24. Simple PID on vertical velocity (`controllers/pid.py`).
      **← next step, in `CODE_TO_BE_REWRITTEN.py`**
- [ ] 25. Closed-loop PID test — vertical motion only, manual tuning.
- [ ] 26. Extending PID to horizontal position (x, y).
- [ ] 27. Closed-loop simulation loop (`simulation.py`): state →
      controller → physics.
- [ ] 28. First version of `metrics.py`: landing error, touchdown
      velocity, fuel consumption.
- [ ] 29. Plot and evaluation of the first full PID landing (3D + time
      series).

*Phase C checkpoint: PID alone guides the rocket to an (imperfect) landing
from start to finish.*

## Phase D — Transition to MPC (CasADi)

- [ ] 30. Symbolic version of the dynamics in CasADi (a separate
      implementation, not imported from `dynamics.py`).
- [ ] 31. RK4 discretization in CasADi (an integrator implementation
      separate from the one in `integrators.py` — important to avoid
      artificially perfect results).
- [ ] 32. Cost function as CasADi expressions (position error, velocity
      error, control, Δu).
- [ ] 33. Building the NLP: decision variables (state+control over the
      horizon), dynamics constraints.
- [ ] 34. Constraints: thrust min/max, tilt angle, altitude ≥ 0, mass ≥
      dry mass.
- [ ] 35. Solving a single open-loop NLP (no MPC loop yet) — trajectory
      preview.
- [ ] 36. Receding-horizon MPC loop: at every step solve the NLP from the
      current state, apply the first control, simulate one step, repeat.
- [ ] 37. Warm-start from the previous solution.
- [ ] 38. Handling solver failures (logging, fallback).
- [ ] 39. First full nominal landing with MPC + plots.

*Phase D checkpoint: MPC alone guides the rocket to a landing in the
nominal scenario.*

## Phase E — Experiments and robustness

- [ ] 40. Standardizing starting scenarios and metric collection for PID
      and MPC.
- [ ] 41. Migrating parameters to `configs/*.yaml` (only now — easy
      scenario switching between experiments).
- [ ] 42. Experiment A: nominal PID vs MPC comparison, results table.
- [ ] 43. Adding wind (`environment.py`) and repeating the experiment.
- [ ] 44. Initial-velocity and mass error — further variants of
      Experiment C.
- [ ] 45. Mismatch between the MPC model and the "real" simulator model
      (different thrust/Isp).
- [ ] 46. Experiment B: grid of initial conditions, success/failure map.
- [ ] 47. Experiment D: MPC parameter sweep (horizon, weights, control
      frequency).
- [ ] 48. Experiment E: Monte Carlo (RNG seed recorded), statistics and
      plots.
- [ ] 49. (Time permitting) aerodynamic drag.
- [ ] 50. (Time permitting) extension to orientation/body tilt angle.

*Phase E checkpoint: a full, repeatable experiment matrix in
`experiments/`.*

## Phase F — Thesis writing and finish

- [ ] 51. Numerical verification (sensitivity to the time step, audit of
      constraint violations).
- [ ] 52. Chapter: methodology and mathematical model.
- [ ] 53. Chapter: MPC formulation and implementation.
- [ ] 54. Chapter: experiments and results (tables/plots from Phase E).
- [ ] 55. Chapter: conclusions and limitations + final `README.md` with
      reproduction instructions.
- [ ] 56. Presentation, demo recording, short technical report in English
      (portfolio).
- [ ] 57. Defense rehearsal: questions about MPC non-convergence,
      insufficient thrust, model verification, the accuracy/computation-
      time trade-off.
