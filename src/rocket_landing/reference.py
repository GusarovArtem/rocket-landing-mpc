import numpy as np

class LandingTarget:
    """
    Defines WHAT a successful landing means, as plain numbers - nothing
    about HOW to get there (that's the controller job, starting next step)
    Every controller we build (PID now, MPC later) needs some fixed goal
    to compare the rocket's current state against, and this is that goal.

   A "soft landing" meants two things at once:
        1. position -> at the landing pad (we put the pad at the origin,
        (0,0,0), matching docs/assumptions.md's coordinate convention:
        landing pad = origin of the coordinate system)
        2. velocity -> zero (not moving at all when touching down -
        a real landing doesn't requiere EXACTLY zro, but zero is the
        simpliest possible target, and "how close to zero is goos
        enough" becomes questions for metrics.py later, step 28)

    Mass is deliberately NOT part of the target - we don't care how much
    fuel is left when landing (just that there's enough, which a constraint
    not a goal).
    """


    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])