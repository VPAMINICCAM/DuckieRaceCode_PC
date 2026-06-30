class PIDController_Enhanced:
    def __init__(self, kp, ki, kd, kff=0.0, bff=0.0,
                 u_min=float('-inf'), u_max=float('inf'),
                 tau_aw=1.0):
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Feedforward coefficients
        self.kff = kff
        self.bff = bff

        # Anti-windup gain (or time constant)
        self.tau_aw = tau_aw

        # Output limits
        self.u_min = u_min
        self.u_max = u_max

        # Internal states
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_meas = 0.0
        self.prev_time = None
        self.u_unsat = 0.0

    def reset(self):
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.u_unsat = 0.0

    def update(self, ref, meas, dt=None, current_time=None):
        error = ref - meas

        # Time handling
        if dt is None:
            if self.prev_time is None:
                self.prev_time = current_time
                return 0.0  # No control output on first call
            dt = current_time - self.prev_time
            self.prev_time = current_time
        if dt <= 0.0:
            return 0.0

        # Derivative term
        # derivative = (error - self.prev_error) / dt
        derivative = (meas - self.prev_meas) / dt

        self.prev_meas = meas
        # Feedforward term (not to be limited)
        u_ff = self.kff * ref + self.bff

        # PID raw output (unsaturated)
        u_pid = self.kp * error + self.ki * self.integrator + self.kd * derivative

        # Saturate only the PID part (allowing FF to pass through clean)
        u_pid_sat = max(self.u_min, min(self.u_max , u_pid))

        # Final control signal
        u = u_ff + u_pid_sat

        # Anti-windup correction
        self.integrator += dt * (error + (u_pid_sat - u_pid) / self.tau_aw)

        # Save state
        self.prev_error = error

        return u
    
