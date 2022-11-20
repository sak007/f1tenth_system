import numpy as np
class PID:
    def __init__(self, kP, kI, kD, max_out, min_out=None):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.max_out = max_out
        if min_out == None:
            self.min_out = -max_out
        else:
            self.min_out = min_out
        self.integral = 0
        self.last_error = 0
        self.max_i = np.radians(3)

    def update(self, error, dt=1):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kP * error + self.kI * self.integral + self.kD * derivative
        output = np.clip(output, self.min_out, self.max_out)
        self.last_error = error
        self.integral = np.clip(self.integral, -self.max_i, self.max_i)
        return output

    def reset(self):
        self.integral = 0
        self.last_error = 0