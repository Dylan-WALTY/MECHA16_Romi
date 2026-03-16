"""
closed_loop_driver.py

This module implements a closed-loop PID controller for robotic systems.

Author: Dylan WALTY
Date: 2026-03-16

The main functionality includes PID control for maintaining desired states in
the system by adjusting output based on error calculations.
"""

class ClosedLoopDriver:
    """
    A class that implements a PID control algorithm for closed-loop systems.

    Attributes:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        setpoint (float): Desired target value.
        integral (float): Accumulated error.
        last_error (float): Previous error value.

    Methods:
        compute_output(current_value): Computes the control output based on the current value.
    """

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0.0
        self.integral = 0.0
        self.last_error = 0.0

    def compute_output(self, current_value):
        """
        Calculates the PID control output based on the current value.

        Args:
            current_value (float): The current measurement of the system.

        Returns:
            float: The computed PID output.
        """
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.last_error
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.last_error = error
        return output

