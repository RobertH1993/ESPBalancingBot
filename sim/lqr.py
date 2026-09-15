"""Discrete LQR from robot.xml, linearized around upright.

Sagittal state (SI):
    z = [pitch, pitch_rate, wheel_vx]
Action:
    u = identical left/right motor ctrl in [-1, 1]

Pitch is the chassis angle, not the accelerometer reading (that is
corrupted by linear accel at the IMU). Wheel_vx is r*(wl+wr)/2.
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
from scipy import linalg

XML_PATH = Path(__file__).with_name("robot.xml")
CONTROL_DT = 0.01
WHEEL_RADIUS = 0.0235
STATE_NAMES = ("pitch", "wpitch", "wheel_vx")


def true_pitch(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    cid = model.body("chassis").id
    R = data.xmat[cid].reshape(3, 3)
    return math.atan2(-R[2, 0], R[2, 2])


def wheel_vx(data: mujoco.MjData) -> float:
    return WHEEL_RADIUS * 0.5 * (data.qvel[6] + data.qvel[7])


def set_equilibrium(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    mujoco.mj_resetData(model, data)
    data.qpos[:] = 0.0
    data.qpos[2] = WHEEL_RADIUS
    data.qpos[3] = 1.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def set_reduced_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pitch: float,
    wpitch: float,
    vx: float,
) -> None:
    set_equilibrium(model, data)
    data.qpos[3] = math.cos(pitch / 2.0)
    data.qpos[5] = math.sin(pitch / 2.0)
    data.qvel[0] = vx
    data.qvel[4] = wpitch
    w = vx / WHEEL_RADIUS
    data.qvel[6] = w
    data.qvel[7] = w
    mujoco.mj_forward(model, data)


def get_reduced_state(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    return np.array(
        [true_pitch(model, data), data.qvel[4], wheel_vx(data)],
        dtype=float,
    )


def _advance(model: mujoco.MjModel, data: mujoco.MjData, u: float, nstep: int) -> None:
    data.ctrl[0] = u
    data.ctrl[1] = u
    for _ in range(nstep):
        mujoco.mj_step(model, data)


def linearize(model: mujoco.MjModel, dt: float = CONTROL_DT) -> tuple[np.ndarray, np.ndarray]:
    data = mujoco.MjData(model)
    nstep = max(1, round(dt / model.opt.timestep))
    n = 3
    # vx eps is large enough to be in rolling (not stiction)
    eps = np.array([1e-3, 1e-3, 5e-2])
    eps_u = 5e-2

    A = np.zeros((n, n))
    for i in range(n):
        z = np.zeros(n)
        z[i] = eps[i]
        set_reduced_state(model, data, *z)
        _advance(model, data, 0.0, nstep)
        zp = get_reduced_state(model, data)

        z[i] = -eps[i]
        set_reduced_state(model, data, *z)
        _advance(model, data, 0.0, nstep)
        zm = get_reduced_state(model, data)
        A[:, i] = (zp - zm) / (2.0 * eps[i])

    set_reduced_state(model, data, 0.0, 0.0, 0.0)
    _advance(model, data, eps_u, nstep)
    zp = get_reduced_state(model, data)
    set_reduced_state(model, data, 0.0, 0.0, 0.0)
    _advance(model, data, -eps_u, nstep)
    zm = get_reduced_state(model, data)
    B = ((zp - zm) / (2.0 * eps_u)).reshape(n, 1)
    return A, B


def dlqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    P = linalg.solve_discrete_are(A, B, Q, R)
    return linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def extract_lqr(
    xml_path: Path | str = XML_PATH,
    q_diag: tuple[float, float, float] = (25.0, 1.5, 8.0),
    r: float = 4.0,
) -> dict:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    A, B = linearize(model)
    Q = np.diag(q_diag)
    R = np.array([[r]], dtype=float)
    K = dlqr(A, B, Q, R)
    return {
        "A": A,
        "B": B,
        "Q": Q,
        "R": R,
        "K": K,
        "eigs_open": np.linalg.eigvals(A),
        "eigs_closed": np.linalg.eigvals(A - B @ K),
        "dt": CONTROL_DT,
        "state_names": STATE_NAMES,
    }


class LqrController:
    def __init__(self, result: dict | None = None):
        self.result = result or extract_lqr()
        self.K = self.result["K"].reshape(-1)

    def reset(self, x: float = 0.0):
        del x

    def command(self, pitch_rad: float, wpitch: float, vx: float) -> tuple[float, float]:
        z = np.array([pitch_rad, wpitch, vx], dtype=float)
        u = float(np.clip(-self.K @ z, -1.0, 1.0))
        return u, u


def _print_result(res: dict) -> None:
    print("Discrete A (dt = {:.3f} s)".format(res["dt"]))
    print(np.array2string(res["A"], precision=5, suppress_small=True))
    print("\nB")
    print(np.array2string(res["B"], precision=5, suppress_small=True))
    print("\nOpen-loop |z|:", np.round(np.abs(res["eigs_open"]), 4))
    print("Closed-loop |z|:", np.round(np.abs(res["eigs_closed"]), 4))
    k = res["K"].reshape(-1)
    print("\nK  (u = -K z,  z = [pitch, wpitch, wheel_vx])")
    for name, ki in zip(res["state_names"], k):
        print(f"  {name:10s}  {ki:10.4f}")
    print("\nC snippet (rad, rad/s, m/s; ctrl in [-1, 1]):")
    print(
        "  float u = -({:.6f}f * pitch_rad + {:.6f}f * wpitch + {:.6f}f * vx);".format(
            k[0], k[1], k[2]
        )
    )


if __name__ == "__main__":
    _print_result(extract_lqr())
