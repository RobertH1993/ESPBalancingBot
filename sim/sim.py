import math
import time
import mujoco
import mujoco.viewer
import numpy as np

from control import RobotController
from lqr import LqrController, true_pitch, wheel_vx


model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

left_motor = model.actuator("left_motor").id
right_motor = model.actuator("right_motor").id
chassis_id = model.body("chassis").id

CONTROL_DT = 0.01
steps_per_control = max(1, round(CONTROL_DT / model.opt.timestep))
control_step = 0
print_step = 0

# Unstable equilibrium: a perfectly upright pose with zero velocity never falls.
INITIAL_TILT_DEG = 0.5
PUSH_FORCE_N = 6.0
PUSH_DURATION_STEPS = 20  # 0.1 s at timestep 0.005

GLFW_RIGHT = 262
GLFW_LEFT = 263
GLFW_DOWN = 264
GLFW_UP = 265


class SimInput:
    push = [0.0, 0.0, 0.0]
    push_steps = 0
    control_on = True
    reset = False
    use_lqr = False


keys = SimInput()
controller = RobotController()
print("Linearizing robot.xml for LQR...")
lqr = LqrController()
print("LQR K =", np.array2string(lqr.K, precision=3))


def set_initial_pose():
    theta = math.radians(INITIAL_TILT_DEG)
    data.qpos[3] = math.cos(theta / 2.0)
    data.qpos[4] = 0.0
    data.qpos[5] = math.sin(theta / 2.0)
    data.qpos[6] = 0.0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


set_initial_pose()
qpos0 = data.qpos.copy()
# One physics step so the accelerometer is not all zeros at Kalman init.
mujoco.mj_step(model, data)


def compute_pitch(accel):
    # Firmware uses atan2(accelY, -accelZ) because their IMU reports gravity
    # as -Z when upright. MuJoCo's accelerometer reports specific force (+Z
    # when upright), so the equivalent is atan2(-accelX, accelZ).
    return math.atan2(-accel[0], accel[2]) * 180.0 / math.pi


def key_callback(keycode):
    if keycode in (ord("d"), ord("D"), GLFW_RIGHT):
        keys.push = [PUSH_FORCE_N, 0.0, 0.0]
        keys.push_steps = PUSH_DURATION_STEPS
        print("Push +X")
    elif keycode in (ord("a"), ord("A"), GLFW_LEFT):
        keys.push = [-PUSH_FORCE_N, 0.0, 0.0]
        keys.push_steps = PUSH_DURATION_STEPS
        print("Push -X")
    elif keycode in (ord("w"), ord("W"), GLFW_UP):
        keys.push = [0.0, PUSH_FORCE_N, 0.0]
        keys.push_steps = PUSH_DURATION_STEPS
        print("Push +Y")
    elif keycode in (ord("s"), ord("S"), GLFW_DOWN):
        keys.push = [0.0, -PUSH_FORCE_N, 0.0]
        keys.push_steps = PUSH_DURATION_STEPS
        print("Push -Y")
    elif keycode in (ord("m"), ord("M")):
        keys.control_on = not keys.control_on
        print("Control", "ON" if keys.control_on else "OFF (motors cut)")
    elif keycode in (ord("r"), ord("R")):
        keys.reset = True
    elif keycode in (ord("l"), ord("L")):
        keys.use_lqr = not keys.use_lqr
        print("Controller", "LQR" if keys.use_lqr else "PID")


print("Keys: A/D of pijlen = duw  |  M = motoren  |  L = LQR/PID  |  R = reset")
print("Viewer: body selecteren, Ctrl+sleep = ook een duw")

controller.enter_balancing(compute_pitch(data.sensor("imu_accel").data))
lqr.reset(data.qpos[0])

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        if keys.reset:
            keys.reset = False
            data.qpos[:] = qpos0
            data.qvel[:] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)
            mujoco.mj_step(model, data)
            keys.control_on = True
            keys.push_steps = 0
            controller.enter_balancing(compute_pitch(data.sensor("imu_accel").data))
            lqr.reset(data.qpos[0])

        data.xfrc_applied[:] = 0.0
        if keys.push_steps > 0:
            data.xfrc_applied[chassis_id, 0:3] = keys.push
            keys.push_steps -= 1

        mujoco.mj_step(model, data)
        control_step += 1

        if control_step >= steps_per_control:
            control_step = 0

            accel = data.sensor("imu_accel").data
            gyro = data.sensor("imu_gyro").data
            pitch = compute_pitch(accel)
            gyro_pitch_dps = gyro[1] * 180.0 / math.pi
            wheel_vel_left = data.sensor("left_wheel_vel").data[0]
            wheel_vel_right = data.sensor("right_wheel_vel").data[0]
            ctrl_l = 0.0
            ctrl_r = 0.0

            if keys.control_on:
                if keys.use_lqr:
                    pitch_rad = true_pitch(model, data)
                    wpitch = data.qvel[4]
                    vx = wheel_vx(data)
                    ctrl_l, ctrl_r = lqr.command(pitch_rad, wpitch, vx)
                else:
                    ctrl_l, ctrl_r = controller.step(
                        pitch, gyro_pitch_dps, wheel_vel_left, wheel_vel_right, CONTROL_DT
                    )
                data.ctrl[left_motor] = ctrl_l
                data.ctrl[right_motor] = ctrl_r
            else:
                data.ctrl[left_motor] = 0.0
                data.ctrl[right_motor] = 0.0

            print_step += 1
            if print_step >= 20:
                print_step = 0
                name = "LQR" if keys.use_lqr else controller.mode
                pwm_l = ctrl_l * 1000.0
                pwm_r = ctrl_r * 1000.0
                print(
                    f"{name:10s}  pitch={pitch:7.2f}  "
                    f"pwm={pwm_l:7.1f}/{pwm_r:7.1f}"
                )

        viewer.sync()
        time.sleep(model.opt.timestep)
