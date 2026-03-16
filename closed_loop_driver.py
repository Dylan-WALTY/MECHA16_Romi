class closed_loop_control:
    '''A class to implement a closed loop controller for the motors. This takes in the gains for the controller 
    and has a method to calculate the control effort based on a reference value and a measurement'''
   
    def __init__(self, K_p: float, K_i: float, K_d: float, K_ff: float):
        '''Initializes a Motor object'''
        
        # Placeholder values for the gains of the controller
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.K_ff = K_ff
        # Placeholder for the set point of the controller.
        self.setPoint = 0 
        
        # Placeholder values for the integral and derivative terms of the controller.
        self.integral = 0
        self.prev_error = 0

        self._first_call = True # Flag to indicate if c_loop is being called for the first time (used for derivative term calculation) 

   
    def set_K_p(self, K_p):
        '''Sets the proportional gain'''
        self.K_p = K_p

    def set_K_i(self, K_i):
        '''Sets the integral gain'''
        self.K_i = K_i

    def set_K_d(self, K_d):
        '''Sets the derivative gain'''
        self.K_d = K_d

    def set_K_ff(self, K_ff):
        '''Sets the feedforward gain'''
        self.K_ff = K_ff
    
    def set_set_point(self, setPoint):
        '''Sets the set point for the closed loop controller'''
        self.setPoint = setPoint

    def reset(self):
        '''Resets the integral and derivative terms of the controller (if needed)'''
        self.integral = 0
        self.prev_error = 0
        self._first_call = True

         
    def c_loop(self, measurement, dt):
        '''Calculates the output of the closed loop controller based on the input measurement and the reference value'''
        error = self.setPoint - measurement

        # Update the integral term with the new error and the time step.
        self.integral += error * dt
        # Implement anti-windup for the integral term by clamping it to a maximum value.
        if self.integral > 100:
            self.integral = 100
        elif self.integral < -100:
            self.integral = -100

        # Calculate the derivative term based on the change in error and the time step.
        if self._first_call or dt <= 0:
            derivative = 0
            self._first_call = False
        else:
            derivative = (error - self.prev_error) / dt
        
        # Update the previous error for the next iteration
        self.prev_error = error

        # Placeholder for the actual control law to convert from velocity error to PWM effort command. You will replace this with your actual control law that uses the error, integral, and derivative terms along with the feedforward term.
        output_v = self.K_p * error + self.integral * self.K_i + derivative * self.K_d + self.K_ff * self.setPoint # Placeholder for actual conversion from velocity to PWM effort

        if output_v > 100:
            output_v = 100
        elif output_v < -100:
            output_v = -100

        return output_v # Placeholder for actual effort correction function