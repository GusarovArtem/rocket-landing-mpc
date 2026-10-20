import numpy as np

from rocket_landing.actuators import clamp_control
from rocket_landing.params import VehicleParams

p = VehicleParams()

# a control that way outside every limit in every direction:
wild_control = np.array([999999.9, 5.0, -5.0])
print(clamp_control(wild_control, p))

# a control thats perfectly legal
legal_control=np.array([200000.0, 0.1, -0.1])
print(clamp_control(legal_control, p))

