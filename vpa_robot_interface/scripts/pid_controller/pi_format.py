#!/usr/bin/python3

class PI_controller:

    """ A class to implement a PI (Proportional-Intergral) controller """

    def __init__(self, kp, ki) -> None:
        self.kp = kp
        self.ki = ki
        self.frequency = 20
        self.dt = 1 / self.frequency

        self.last_error = 0
        self.last_output     = 0

        self.debug_probe_p = 0
        self.debug_probe_i = 0

    def pi_control(self, ref, sig, turn_off_zero_ref:bool) -> float:
        if ref == 0 and turn_off_zero_ref:
            self.reset_controller()
            return 0
        
        # current err
        err = ref - sig

        delta_err = err - self.last_error
        
        delta_output = self.kp * delta_err + self.ki * self.dt * err
        self.debug_probe_p = self.kp * delta_err
        self.debug_probe_i = self.ki * self.dt * err
        output = self.last_output + delta_output

        self.last_error = err
        self.last_output = output

        return output
    
    def update_controller_param(self,kp,ki) -> None:
        self.kp = kp
        self.ki = ki
        
    def reset_controller(self):        
        self.last_error = 0
        self.last_output = 0       

    def return_debug(self) -> tuple:
        return (self.debug_probe_p, self.debug_probe_i)