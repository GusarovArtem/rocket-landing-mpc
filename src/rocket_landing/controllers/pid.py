from scipy.differentiate import derivative


class PIDController:
    """
    A classic PID controller - standalone and generic, not specific to
    rockets. It knows nothing about thrust, vz, or landing: you give it
    an 'error' (how much off you currently are from what you want) and a
    'dt' (how much time passed since the last call), and it hands back
    one number - the "correction" - using the three classic terms:


        P (proportional) -> kp * error
            "the bigger the error RIGHT now, the bigger the correction"
        I (integral)     -> ki * (running total of error over time)
            "if the error has been sticking around for a while instead
            of going away, push harder" - this is what final removes
            small steady leftover errors that P alone can never fix
        D (derivative)    -> kd * (how fast the error is changing)
            "if the error is already shrinking fast on its own, don't
            overreact / slam the brakes" - this is what prevents
            overshoot and oscillation
    """


    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        # None means "no prev error yet" - we use this to skip the
        # derivative term on the very first call (there's no "previous"
        # to compare against yet, so don't guess at a rate of change).
        self.prev_error = None

    def compute(self, error, dt):
        self.integral += error * dt

        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error
        return  self.kp * error + self.ki * self.integral + self.kd * derivative