import numpy as np

from rocket_landing.controllers.pid import PIDController

pid = PIDController(kp=2.0, ki=0.5, kd=0.1)


# Feed is SAME 3 error 3 imes in a roe (as if vz were stuck,
# no improving at all) - this maked it easy to hand-check both
# the integral term (should grow by a fixed amount each call) and the
# derivative term (should be exactly 0 after the first call,
# since a REPEATED error isn't changing).
print(pid.compute(error=5.0, dt=0.1))
print(pid.compute(error=5.0, dt=0.1))
print(pid.compute(error=5.0, dt=0.1))
