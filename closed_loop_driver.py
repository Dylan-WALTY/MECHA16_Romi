class closed_loop_control:
    """Generic PID + feedforward controller.

    Computes a signed output in [-100, 100] from proportional, integral,
    derivative, and feedforward terms. Anti-windup clamps the integral
    accumulator to ±100. The derivative term is suppressed on the first
    call to avoid a spike from an undefined previous error.
    """

    def __init__(self, K_p: float, K_i: float, K_d: float, K_ff: float):
        """Initialise gains and reset all accumulator state."""
        self.K_p  = K_p
        self.K_i  = K_i
        self.K_d  = K_d
        self.K_ff = K_ff

        self.setPoint   = 0
        self.integral   = 0
        self.prev_error = 0
        self._first_call = True

    def set_K_p(self, K_p):
        """Update the proportional gain."""
        self.K_p = K_p

    def set_K_i(self, K_i):
        """Update the integral gain."""
        self.K_i = K_i

    def set_K_d(self, K_d):
        """Update the derivative gain."""
        self.K_d = K_d

    def set_K_ff(self, K_ff):
        """Update the feedforward gain."""
        self.K_ff = K_ff

    def set_set_point(self, setPoint):
        """Update the reference setpoint."""
        self.setPoint = setPoint

    def reset(self):
        """Clear integral accumulator, previous error, and first-call flag."""
        self.integral    = 0
        self.prev_error  = 0
        self._first_call = True

    def c_loop(self, measurement, dt):
        """Compute one control update and return the clamped output in [-100, 100].

        Args:
            measurement: Current process value (e.g. wheel velocity in mm/s).
            dt:          Elapsed time since last call in seconds.
        """
        error = self.setPoint - measurement

        self.integral += error * dt
        self.integral  = max(-100, min(100, self.integral))

        if self._first_call or dt <= 0:
            derivative       = 0
            self._first_call = False
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        output = (self.K_p  * error
                + self.K_i  * self.integral
                + self.K_d  * derivative
                + self.K_ff * self.setPoint)

        return max(-100, min(100, output))
