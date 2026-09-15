# Port of the control task in code/balancingbot/main/main.c

import math

from kallman import KallmanFilter
from pid import PidController

MODE_SETTLING = "SETTLING"
MODE_BALANCING = "BALANCING"
MODE_FALLEN = "FALLEN"

SPEED_FILTER_ALPHA = 0.98
MAX_ANGLE_BEFORE_STOP = 42.0
STEADY_ANGLE_THRESHOLD = 0.45
PWM_MAX = 1000.0

# Firmware: speed = pulses * WHEEL_CM_PER_ENCODER_TICK / dt
# MG310P20: 13-line Hall, 20:1 gearbox, PCNT 4x. Full 4x (1040 counts/rev)
# saturates the speed PID on this plant; 2x matches AB-edge counting and
# makes the outer loop as stiff as the real robot (almost still).
# Sign is opposite PWM: with Kp_speed = -0.7 that is negative feedback.
WHEEL_CM_PER_ENCODER_TICK = 0.00571428571
ENCODER_COUNTS_PER_REV = 13 * 20 * 2
ENCODER_SIGN = -1.0


def wheel_speed_cm_s(omega_rad_s):
    return (
        ENCODER_SIGN
        * omega_rad_s
        * ENCODER_COUNTS_PER_REV
        * WHEEL_CM_PER_ENCODER_TICK
        / (2.0 * math.pi)
    )


class RobotController:
    def __init__(self):
        self.mode = MODE_SETTLING
        self.kf = KallmanFilter(0.0)
        self.speed_filtered = 0.0
        self.counter = 0
        self.settling_counter = 0
        self.distance_left = 0.0
        self.distance_right = 0.0
        self.target_speed = 0.0
        self.target_turn_rate = 0.0
        self.pwm_left = 0.0
        self.pwm_right = 0.0

        self.pid_balance = PidController(kp=-75.0, ki=0.0, kd=-1.0, setpoint=0.0, max_output=1000.0)
        self.pid_speed = PidController(kp=-0.7, ki=-0.25, kd=0.02, setpoint=0.0, max_output=35.0)
        self.pid_trim = PidController(kp=0.8, ki=0.1, kd=0.0, setpoint=0.0, max_output=150.0)

    def stop_motors(self):
        self.pwm_left = 0.0
        self.pwm_right = 0.0

    def enter_balancing(self, pitch):
        self.kf.init(pitch)
        self.speed_filtered = 0.0
        self.counter = 0
        self.pid_balance.reset(0.0)
        self.pid_speed.reset(0.0)
        self.pid_trim.reset(0.0)
        self.distance_left = 0.0
        self.distance_right = 0.0
        print("Settled, start balancing")
        self.mode = MODE_BALANCING

    def handle_settling(self, pitch):
        self.stop_motors()
        if abs(pitch) <= STEADY_ANGLE_THRESHOLD:
            self.settling_counter += 1
            if self.settling_counter >= 3:
                self.enter_balancing(pitch)
                self.settling_counter = 0
        else:
            self.settling_counter = 0

    def handle_balancing(self, pitch, gyro_pitch_dps, wheel_vel_left, wheel_vel_right, dt):
        # Firmware Kalman uses -gyroX; D-term uses +gyroX. gyroX is opposite
        # our MuJoCo pitch rate, so Kalman gets +rate and D gets -rate.
        filtered_angle = self.kf.update(pitch, gyro_pitch_dps, dt)

        if abs(filtered_angle) >= MAX_ANGLE_BEFORE_STOP:
            print("Pitch is too high, robot fell over")
            self.stop_motors()
            self.mode = MODE_FALLEN
            return

        speed_left = wheel_speed_cm_s(wheel_vel_left)
        speed_right = wheel_speed_cm_s(wheel_vel_right)
        self.distance_left += speed_left
        self.distance_right += speed_right

        self.speed_filtered = (
            SPEED_FILTER_ALPHA * ((speed_left + speed_right) / 2.0)
            + (1.0 - SPEED_FILTER_ALPHA) * self.speed_filtered
        )

        if self.counter == 9:
            self.pid_speed.setpoint = self.target_speed
            new_setpoint = self.pid_speed.compute(self.speed_filtered, dt * 10.0, 0.0)

            difference = new_setpoint - self.pid_balance.setpoint
            alpha = 0.25
            if abs(difference) >= 5.0:
                alpha = 0.5
            self.pid_balance.setpoint = (
                self.pid_balance.setpoint * alpha + new_setpoint * (1.0 - alpha)
            )
            self.counter = 0
        self.counter += 1

        pid_output = self.pid_balance.compute(filtered_angle, dt, -gyro_pitch_dps)

        self.pid_trim.setpoint = self.target_turn_rate
        wheel_trim = self.pid_trim.compute(
            self.distance_left - self.distance_right, dt, 0.0
        )

        self.pwm_left = pid_output - wheel_trim
        self.pwm_right = pid_output + wheel_trim

    def handle_fallen(self, pitch):
        self.stop_motors()
        if abs(pitch) <= STEADY_ANGLE_THRESHOLD:
            print("Back upright, settling before balancing again")
            self.mode = MODE_SETTLING

    def step(self, pitch, gyro_pitch_dps, wheel_vel_left, wheel_vel_right, dt):
        if self.mode == MODE_SETTLING:
            self.handle_settling(pitch)
        elif self.mode == MODE_BALANCING:
            self.handle_balancing(pitch, gyro_pitch_dps, wheel_vel_left, wheel_vel_right, dt)
        elif self.mode == MODE_FALLEN:
            self.handle_fallen(pitch)

        return self.pwm_left / PWM_MAX, self.pwm_right / PWM_MAX
