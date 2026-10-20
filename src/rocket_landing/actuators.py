import numpy as np

def clamp_control(control, params):
    """
    Takes a RAW control command [thrust, alpha, beta] - which might come
    from a sloppy hand-written test, a not-yet-trained controller, or an
    optimizer that momentarily asked for something unrealistic - and
    returns a NEW control vector that respectts the physical limits of
    the engine and the steering mechanism:


        thrust  -> clipped to [params.min_thrust, params.max_thrust]
        alpha   -> clipped to [-params.max_tilt_angle, +params.max_tilt_angle]
        beta    -> clipped to [-params.max_tilt_angle, +params.max_tilt_angle]
    """
    thrust, alpha, beta = control

    clamped_thrust = np.clip(thrust, params.min_thrust, params.max_thrust)
    clamped_alpha = np.clip(alpha, -params.max_tilt_angle, params.max_tilt_angle)
    clamped_beta = np.clip(beta, -params.max_tilt_angle, params.max_tilt_angle)

    return np.array([clamped_thrust, clamped_alpha, clamped_beta])
