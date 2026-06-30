"""
A Feedforward PI (Proportional-Integral) Controller with optional feedforward term and anti-windup mechanism.
Attributes:
    kp (float): Proportional gain.
    ki (float): Integral gain.
    kff (float): Feedforward gain. Default is 0.
    bff (float): Feedforward bias. Default is 0.
    integral (float): Accumulated integral term.
    previous_error (float): Previous error value.
    integral_limit (float or None): Limit for the integral term to prevent windup. Default is None.
    output_limit (float or None): Limit for the control signal output to prevent saturation. Default is None.
Methods:
    __init__(kp, ki, kff=0, bff=0, integral_limit=None, output_limit=None):
        Initializes the controller with the given gains and optional feedforward terms.
    reset():
        Resets the integral and previous error terms to zero.
    update(setpoint, measurement, dt):
        Updates the control signal based on the setpoint, measurement, and time step.
        Implements anti-windup by clamping the integral term if integral_limit is set.
        Implements output saturation by clamping the control signal if output_limit is set.
Example:
    To create a controller with an integral limit of 10 and output limit of 100:
    controller = FeedforwardPIController(kp=1.0, ki=0.1, integral_limit=10, output_limit=100)
"""

class FeedforwardPIController:
    def __init__(self, kp, ki, kff=0, bff=0, integral_limit=None, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kff = kff
        self.bff = bff
        self.integral = 0
        self.previous_error = 0
        self.integral_limit = integral_limit
        self.output_limit = output_limit

    def reset(self):
        self.integral = 0
        self.previous_error = 0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        delta_error = error - self.previous_error
        self.previous_error = error
        
        self.integral += error * dt
        
        # Anti-windup: Clamp the integral term
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        
        feedforward = self.kff * setpoint + self.bff
        control_signal = self.kp * delta_error + self.ki * self.integral + feedforward
        
        # Output saturation: Clamp the control signal
        if self.output_limit is not None:
            control_signal = max(min(control_signal, self.output_limit), -self.output_limit)
        
        return control_signal
    
    def changeparam(self, kp=None, ki=None, kff=None, bff=None, integral_limit=None, output_limit=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kff is not None:
            self.kff = kff
        if bff is not None:
            self.bff = bff
        if integral_limit is not None:
            self.integral_limit = integral_limit
        if output_limit is not None:
            self.output_limit = output_limit