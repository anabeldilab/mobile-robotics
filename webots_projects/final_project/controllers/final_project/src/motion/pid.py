"""PID controller implementation.

This module contains the PID class, which is a simple implementation of a
Proportional-Integral-Derivative controller. The PID controller is used to
control the speed of the robot's wheels based on the error between the desired 
speed and the actual speed.

Example:

    pid = PID(1.0, 0.1, 0.01)

    # Controlador P
    output_p = pid.compute(10, 0.1, mode='P')

    # Controlador PI
    output_pi = pid.compute(10, 0.1, mode='PI')

    # Controlador PID
    output_pid = pid.compute(10, 0.1, mode='PID')
    
    Attributes:
    - kp: Proportional gain
    - ki: Integral gain
    - kd: Derivative gain
    - previous_error: Error from the previous iteration
    - integral: Integral of the error over time
    
    Methods:
    - compute: Compute the output of the PID controller based on the error between
               the setpoint and the measured value.
    
"""

class PID:
    def __init__(self, kp, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, target, measured_value):
        error = target - measured_value
        output = 0

        self.integral += error
        derivative = error - self.previous_error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output