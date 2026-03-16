# motor_driver.py

"""
@file    motor_driver.py
@brief   This module provides control algorithms for the MECHA16 Romi robot.
@author  Dylan-WALTY
@date    2026-03-16

This module includes classes and methods to manage the motor driver functionality, enabling motion control and operation of the MECHA16 Romi robot.
"""

class MotorDriver:
    """
    @brief   Class to control the motors of the MECHA16 Romi robot.
    @param   None
    @return  None
    """

    def __init__(self, motor1, motor2):
        """
        @brief   Initializes the MotorDriver with two motors.
        @param   motor1: The first motor to control.
        @param   motor2: The second motor to control.
        @return  None
        """
        self.motor1 = motor1
        self.motor2 = motor2

    def set_speed(self, speed1, speed2):
        """
        @brief   Sets the speed for each motor.
        @param   speed1: Speed for motor1.
        @param   speed2: Speed for motor2.
        @return  None
        """
        self.motor1.set_speed(speed1)
        self.motor2.set_speed(speed2)

    def stop(self):
        """
        @brief   Stops both motors.
        @param   None
        @return  None
        """
        self.motor1.stop()
        self.motor2.stop()

# Additional functionality can be added as needed.