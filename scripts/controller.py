class PID:
    def __init__(self, kp, ki, kd, umin=-1.0, umax=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.umin, self.umax = umin, umax
        self.ei, self.e_prev = 0.0, 0.0

    def step(self, e, dt):
        self.ei += e*dt
        ed = (e - self.e_prev)/dt if dt>0 else 0.0
        u = self.kp*e + self.ki*self.ei + self.kd*ed
        self.e_prev = e
        return max(self.umin, min(self.umax, u))