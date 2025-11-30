class PID:
    def __init__(self, kp, ki, kd, umin=-1.0, umax=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.umin, self.umax = umin, umax
        self.ei = 0.0
        self.e_prev = None  # Avoid derivative kick on first step

    def reset(self):
        self.ei = 0.0
        self.e_prev = None

    def step(self, e, dt):
        # Integrator
        self.ei += e * dt
        # Derivative (avoid initial kick)
        if self.e_prev is None or dt <= 0:
            ed = 0.0
        else:
            ed = (e - self.e_prev) / dt
        # PID output
        u = self.kp * e + self.ki * self.ei + self.kd * ed
        self.e_prev = e
        # Saturation
        return max(self.umin, min(self.umax, u))