# Line-for-line port of code/balancingbot/main/kallman.c
# 2-state Kalman filter: angle (deg) + gyro bias (deg/s)


class KallmanFilter:
    def __init__(self, initial_angle=0.0):
        self.init(initial_angle)

    def init(self, initial_angle):
        # Reasonable defaults for a MEMS accel/gyro combo
        self.Q_angle = 0.00001
        self.Q_bias = 0.001
        self.R_measure = 0.5

        self.angle = initial_angle
        self.bias = 0.0

        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, new_angle, new_rate, dt):
        # Predict: integrate the bias-corrected gyro rate
        rate = new_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Update: correct using the accelerometer-derived angle
        S = self.P[0][0] + self.R_measure
        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        y = new_angle - self.angle

        self.angle += K0 * y
        self.bias += K1 * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K0 * P00_temp
        self.P[0][1] -= K0 * P01_temp
        self.P[1][0] -= K1 * P00_temp
        self.P[1][1] -= K1 * P01_temp

        return self.angle

    def set_angle_variance(self, q_angle):
        self.Q_angle = q_angle

    def set_bias_variance(self, q_bias):
        self.Q_bias = q_bias

    def set_measurement_variance(self, r_measure):
        self.R_measure = r_measure
