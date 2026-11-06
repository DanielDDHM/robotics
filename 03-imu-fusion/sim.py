"""Experimentos da licao 03.

Uso:
  python sim.py compare                          # 4 estimadores, 8 voltas cada
  python sim.py drive --estimator kalman         # um estimador so
  mjpython sim.py drive --estimator gyro --view  # em 3D
"""

import argparse
import math
import sys
import time
from pathlib import Path

# Reaproveita o robo (licao 01) e o go-to-goal (licao 02)
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "01-diff-drive"))
sys.path.insert(0, str(ROOT / "02-go-to-goal"))

import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from controllers import WheelSpeedController, normalize_angle  # noqa: E402
from estimators import ComplementaryOdometry, GyroOdometry, KalmanOdometry, WheelOdometry  # noqa: E402
from navigation import GoToGoal, unicycle_to_wheels  # noqa: E402
from robot import CONTROL_HZ, TICKS_PER_REV, WHEEL_BASE, WHEEL_RADIUS, SimRobot  # noqa: E402

MAX_WHEEL_SPEED = 0.35  # m/s
WAYPOINTS = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
LAPS = 8
ESTIMATORS = {
    "odom": WheelOdometry,
    "gyro": GyroOdometry,
    "complementary": ComplementaryOdometry,
    "kalman": KalmanOdometry,
}


class Run:
    """Robo + PID das rodas + estimador + go-to-goal. A navegacao so enxerga o estimador."""

    def __init__(self, estimator_cls, view: bool = False):
        self.robot = SimRobot()
        self.wheels = WheelSpeedController(self.robot)
        # Wheel base NOMINAL de proposito: nada de calibrar o piso nesta licao
        self.est = estimator_cls(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REV)
        self.nav = GoToGoal()
        self.viewer = make_viewer(self.robot) if view else None
        # (t, x, y, th reais, x, y, th estimados, bias real, bias estimado, theta_std, wheel base estimado)
        self.log = []
        self.alive = True

    def _step(self, v: float, omega: float) -> None:
        self.wheels.update(*unicycle_to_wheels(v, omega, WHEEL_BASE, MAX_WHEEL_SPEED))
        self.est.update(*self.wheels.d_ticks, self.robot.read_gyro(), self.robot.dt)
        self.log.append((self.robot.time, *self.robot.ground_truth_pose(),
                         self.est.x, self.est.y, self.est.theta,
                         self.robot.gyro_bias(), self.est.bias, getattr(self.est, "theta_std", np.nan),
                         getattr(self.est, "effective_wheel_base", np.nan)))
        self.robot.step()
        if self.viewer is not None:
            self.viewer.sync()
            time.sleep(self.robot.dt)
            self.alive = self.viewer.is_running()

    def wait(self, seconds: float) -> None:
        for _ in range(int(seconds * CONTROL_HZ)):
            if self.alive:
                self._step(0.0, 0.0)

    def drive_to(self, goal, timeout: float = 20.0) -> None:
        for _ in range(int(timeout * CONTROL_HZ)):
            v, omega, reached = self.nav.command((self.est.x, self.est.y, self.est.theta), goal)
            if reached or not self.alive:
                return
            self._step(v, omega)

    def laps(self, n: int) -> list[float]:
        """Fica 2 s parado (calibracao do gyro) e da n voltas. Retorna o erro real ao fim de cada volta."""
        self.wait(2.0)
        errors = []
        for _ in range(n):
            for wp in WAYPOINTS:
                self.drive_to(wp)
            x, y, _ = self.robot.ground_truth_pose()
            errors.append(math.hypot(x, y))
        return errors

    def array(self) -> np.ndarray:
        return np.array(self.log).T


def make_viewer(robot: SimRobot):
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(robot.model, robot.data)
    viewer.cam.distance, viewer.cam.elevation = 3.0, -55
    viewer.cam.lookat[:] = (0.4, 0.4, 0)
    with viewer.lock():
        for i, (x, y) in enumerate(WAYPOINTS):
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_CYLINDER,
                np.array([0.04, 0.002, 0.0]), np.array([x, y, 0.002]),
                np.eye(3).flatten(), np.array([0.2, 0.9, 0.3, 0.7], dtype=np.float32),
            )
        viewer.user_scn.ngeom = len(WAYPOINTS)
    return viewer


def heading_error(log: np.ndarray) -> np.ndarray:
    return np.degrees([normalize_angle(a - b) for a, b in zip(log[3], log[6])])


def run_compare(view: bool) -> None:
    results = {}
    for key, cls in ESTIMATORS.items():
        run = Run(cls)
        errors = run.laps(LAPS)
        results[key] = (cls.name, errors, run.array())
        print(f"{cls.name:28s} erro por volta (cm): " + " ".join(f"{e * 100:5.1f}" for e in errors))

    # Trajetorias
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    for ax, (name, errors, log) in zip(axes.flat, results.values()):
        ax.plot(log[1], log[2], lw=0.8, label="real")
        ax.plot(log[4], log[5], "--", lw=0.8, label="estimado")
        ax.plot(*zip(*WAYPOINTS), "g*", ms=12)
        ax.set_aspect("equal")
        ax.set_xlim(-0.6, 1.6)
        ax.set_ylim(-0.6, 1.6)
        ax.set_title(f"{name}: {errors[-1] * 100:.1f} cm apos {LAPS} voltas")
        ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig("trajectories.png", dpi=100)

    # Erro por volta e erro de angulo ao longo do tempo
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    for name, errors, log in results.values():
        ax1.plot(range(1, LAPS + 1), np.array(errors) * 100, "o-", label=name)
        ax2.plot(log[0], heading_error(log), label=name)
    ax1.set_xlabel("volta")
    ax1.set_ylabel("distancia real da origem (cm)")
    ax1.set_title("Erro ao fim de cada volta")
    ax1.legend()
    ax2.set_xlabel("tempo (s)")
    ax2.set_ylabel("erro de angulo (graus)")
    ax2.set_title("Angulo real menos angulo estimado")
    ax2.axhline(0, color="k", lw=0.5)
    ax2.legend()
    fig.tight_layout()
    fig.savefig("errors.png", dpi=100)

    # O que os filtros aprendem: bias do gyro e wheel base efetivo
    fig, (ax, ax_l) = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
    log = results["kalman"][2]
    ax.plot(log[0], np.degrees(log[7]), "k", lw=2, label="bias real")
    for key in ("gyro", "complementary", "kalman"):
        name, _, log = results[key]
        ax.plot(log[0], np.degrees(log[8]), label=name)
    ax.set_ylabel("bias do gyro (graus/s)")
    ax.set_title("Rastreando o bias do giroscopio enquanto ele aquece")
    ax.legend()
    ax.set_ylim(np.degrees(log[7]).min() - 0.4, np.degrees(log[7]).max() + 0.4)
    ax_l.plot(log[0], log[10] * 100, color="tab:red", label="EKF")
    ax_l.axhline(WHEEL_BASE * 100, color="k", ls=":", label="nominal (18,00 cm)")
    ax_l.axhline(18.42, color="tab:green", ls="--", label="calibracao manual da licao 02 (18,42 cm)")
    ax_l.set_ylim(17.8, 18.8)
    ax_l.set_xlabel("tempo (s)")
    ax_l.set_ylabel("wheel base (cm)")
    ax_l.set_title("Wheel base efetivo aprendido pelo EKF em movimento")
    ax_l.legend()
    fig.tight_layout()
    fig.savefig("bias.png", dpi=100)
    print("Graficos salvos: trajectories.png, errors.png, bias.png")


def run_drive(view: bool, key: str) -> None:
    run = Run(ESTIMATORS[key], view)
    errors = run.laps(LAPS)
    print(f"{ESTIMATORS[key].name}: erro por volta (cm): " + " ".join(f"{e * 100:.1f}" for e in errors))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", choices=["compare", "drive"])
    parser.add_argument("--estimator", choices=ESTIMATORS, default="kalman")
    parser.add_argument("--view", action="store_true", help="abre o viewer 3D (use mjpython no macOS)")
    args = parser.parse_args()
    if args.experiment == "compare":
        run_compare(args.view)
    else:
        run_drive(args.view, args.estimator)
