# Line-for-line port of code/balancingbot/main/pid.c


class PidController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, setpoint=0.0, max_output=0.0):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.output = 0.0
        self.max_output = max_output
        self.setpoint = setpoint
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, measurement, dt, external_d=0.0):
        error = self.setpoint - measurement

        self.P = self.Kp * error
        self.integral += error * dt
        self.I = self.Ki * self.integral

        if external_d != 0.0:
            self.D = self.Kd * external_d
        else:
            self.D = self.Kd * (error - self.prev_error) / dt
            self.prev_error = error

        self.output = self.P + self.I + self.D

        if self.output > self.max_output:
            self.output = self.max_output
            self.integral -= error * dt
        elif self.output < -self.max_output:
            self.output = -self.max_output
            self.integral -= error * dt

        return self.output

    def reset(self, setpoint):
        self.output = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.setpoint = setpoint
