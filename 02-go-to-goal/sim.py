"""Experimentos da licao 02.

Uso:
  python sim.py goal [--x 1.0 --y 0.6]   # vai ate um ponto
  python sim.py gains                    # compara ganhos de giro (k_alpha)
  python sim.py calibrate                # mede o wheel base efetivo girando no lugar
  python sim.py square [--wheel-base L]  # 3 voltas num quadrado guiado pela odometria
  mjpython sim.py square --view          # qualquer experimento com --view abre o 3D
"""

import argparse
import math
import sys
import time
from pathlib import Path

# Reaproveita o robo e os controladores da licao 01
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "01-diff-drive"))

import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from controllers import Odometry, WheelSpeedController, normalize_angle  # noqa: E402
from navigation import GoToGoal, unicycle_to_wheels  # noqa: E402
from robot import CONTROL_HZ, TICKS_PER_REV, WHEEL_BASE, WHEEL_RADIUS, SimRobot  # noqa: E402

MAX_WHEEL_SPEED = 0.35  # m/s


class Run:
    """Um robo simulado com toda a pilha montada: PID das rodas, odometria e go-to-goal."""

    def __init__(self, nav: GoToGoal, view: bool = False, markers=(), odom_wheel_base: float = WHEEL_BASE):
        self.robot = SimRobot()
        self.wheels = WheelSpeedController(self.robot)
        self.odom = Odometry(WHEEL_RADIUS, odom_wheel_base, TICKS_PER_REV)
        self.nav = nav
        self.viewer = make_viewer(self.robot, markers) if view else None
        self.log = []  # (t, x_real, y_real, th_real, x_odom, y_odom, th_odom, v, omega)
        self.alive = True

    def _step(self, v: float, omega: float) -> None:
        self.wheels.update(*unicycle_to_wheels(v, omega, WHEEL_BASE, MAX_WHEEL_SPEED))
        self.odom.update(*self.wheels.d_ticks)
        self.log.append((self.robot.time, *self.robot.ground_truth_pose(),
                         self.odom.x, self.odom.y, self.odom.theta, v, omega))
        self.robot.step()
        if self.viewer is not None:
            self.viewer.sync()
            time.sleep(self.robot.dt)
            self.alive = self.viewer.is_running()

    def drive_to(self, goal: tuple[float, float], timeout: float = 20.0) -> bool:
        """Navega ate o alvo usando SO a odometria como referencia de posicao."""
        for _ in range(int(timeout * CONTROL_HZ)):
            pose = (self.odom.x, self.odom.y, self.odom.theta)
            v, omega, reached = self.nav.command(pose, goal)
            if reached or not self.alive:
                return reached
            self._step(v, omega)
        return False

    def stop(self, seconds: float = 0.5) -> None:
        for _ in range(int(seconds * CONTROL_HZ)):
            if not self.alive:
                return
            self._step(0.0, 0.0)

    def array(self) -> np.ndarray:
        return np.array(self.log).T


def make_viewer(robot: SimRobot, markers):
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(robot.model, robot.data)
    viewer.cam.distance, viewer.cam.elevation = 3.0, -55
    viewer.cam.lookat[:] = (0.4, 0.4, 0)
    # Discos verdes no chao marcando os alvos (so visual, nao colidem)
    with viewer.lock():
        for i, (x, y) in enumerate(markers):
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[i], mujoco.mjtGeom.mjGEOM_CYLINDER,
                np.array([0.04, 0.002, 0.0]), np.array([x, y, 0.002]),
                np.eye(3).flatten(), np.array([0.2, 0.9, 0.3, 0.7], dtype=np.float32),
            )
        viewer.user_scn.ngeom = len(markers)
    return viewer


def run_goal(view: bool, goal: tuple[float, float]) -> None:
    run = Run(GoToGoal(), view, markers=[goal])
    reached = run.drive_to(goal)
    run.stop()
    t, xr, yr, thr, xo, yo, tho, v, omega = run.array()

    fig, (ax, ax_v, ax_w) = plt.subplots(1, 3, figsize=(15, 5), gridspec_kw={"width_ratios": [1.3, 1, 1]})
    ax.plot(xr, yr, label="real")
    ax.plot(xo, yo, "--", label="odometria")
    ax.plot(*goal, "g*", ms=15, label="alvo")
    ax.set_aspect("equal")
    ax.set_title("Trajetoria")
    ax.legend()
    ax_v.plot(t, v)
    ax_v.set_title("v (m/s): acelera, cruza, desacelera")
    ax_v.set_xlabel("tempo (s)")
    ax_w.plot(t, np.degrees(omega))
    ax_w.set_title("omega (graus/s): gira primeiro, depois corrige")
    ax_w.set_xlabel("tempo (s)")
    fig.tight_layout()
    fig.savefig("goal.png", dpi=110)
    print("Grafico salvo em goal.png")
    print(f"Chegou: {reached} em {t[-1]:.1f}s")
    print(f"Onde a odometria acha que parou: ({xo[-1]:+.3f}, {yo[-1]:+.3f})")
    print(f"Onde o robo realmente parou:     ({xr[-1]:+.3f}, {yr[-1]:+.3f})")
    print(f"Erro real ate o alvo: {math.hypot(xr[-1] - goal[0], yr[-1] - goal[1]) * 100:.1f} cm")


def run_gains(view: bool) -> None:
    """Alvo a 90 graus para a esquerda: o robo precisa girar E andar."""
    goal = (0.3, 1.0)
    fig, ax = plt.subplots(figsize=(8, 7))
    for k_alpha in (0.8, 4.0, 25.0):
        run = Run(GoToGoal(k_alpha=k_alpha))
        reached = run.drive_to(goal)
        t, xr, yr, *_ = run.array()
        ax.plot(xr, yr, label=f"k_alpha={k_alpha} ({t[-1]:.1f}s)")
        print(f"k_alpha={k_alpha:5.1f}  chegou={reached}  tempo={t[-1]:.1f}s")
    ax.plot(*goal, "g*", ms=15, label="alvo")
    ax.plot(0, 0, "ko", label="inicio (olhando para +x)")
    ax.set_aspect("equal")
    ax.legend()
    ax.set_title("Efeito do ganho de giro")
    fig.tight_layout()
    fig.savefig("gains.png", dpi=110)
    print("Grafico salvo em gains.png")


def run_calibrate(view: bool) -> None:
    """Calibra o wheel base efetivo girando no lugar.

    Num robo real: marque a direcao inicial no chao com fita, mande girar 10 voltas
    pela odometria e meca com um transferidor quanto ele realmente girou.
    Aqui a "fita no chao" e o ground truth do simulador.
    """
    turns = 10
    run = Run(GoToGoal(), view)
    real_start = run.robot.ground_truth_pose()[2]
    odom_total = real_total = 0.0
    prev_odom, prev_real = run.odom.theta, real_start
    turn_rate = 1.5  # rad/s
    while odom_total < turns * 2 * math.pi and run.alive:
        run._step(0.0, turn_rate)
        odom_total += normalize_angle(run.odom.theta - prev_odom)
        real_total += normalize_angle(run.robot.ground_truth_pose()[2] - prev_real)
        prev_odom, prev_real = run.odom.theta, run.robot.ground_truth_pose()[2]
    run.stop()

    # A odometria calcula d_theta = (d_dir - d_esq) / L. As rodas andaram o mesmo
    # tanto nos dois casos, entao o angulo e inversamente proporcional a L.
    effective = WHEEL_BASE * odom_total / real_total
    print(f"Odometria acha que girou: {math.degrees(odom_total):.1f} graus ({odom_total / (2 * math.pi):.2f} voltas)")
    print(f"Robo realmente girou:     {math.degrees(real_total):.1f} graus ({real_total / (2 * math.pi):.2f} voltas)")
    print(f"Wheel base nominal: {WHEEL_BASE:.4f} m   efetivo: {effective:.4f} m")
    print(f"Use: python sim.py square --wheel-base {effective:.4f}")


def run_square(view: bool, odom_wheel_base: float) -> None:
    waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    laps = 3
    run = Run(GoToGoal(), view, markers=waypoints, odom_wheel_base=odom_wheel_base)
    lap_errors = []
    for _ in range(laps):
        for wp in waypoints:
            run.drive_to(wp)
        x, y, _ = run.robot.ground_truth_pose()
        lap_errors.append(math.hypot(x, y))
    run.stop()
    t, xr, yr, thr, xo, yo, tho, *_ = run.array()

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xr, yr, label="real")
    ax.plot(xo, yo, "--", label="odometria")
    ax.plot(*zip(*waypoints), "g*", ms=15, label="waypoints")
    ax.set_aspect("equal")
    ax.legend()
    ax.set_title(f"{laps} voltas guiadas pela odometria (wheel base {odom_wheel_base:.4f} m)")
    fig.tight_layout()
    fig.savefig("square.png", dpi=110)
    print("Grafico salvo em square.png")
    for i, err in enumerate(lap_errors, 1):
        print(f"Fim da volta {i}: robo real a {err * 100:.1f} cm da origem")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", choices=["goal", "gains", "calibrate", "square"])
    parser.add_argument("--view", action="store_true", help="abre o viewer 3D (use mjpython no macOS)")
    parser.add_argument("--x", type=float, default=1.0)
    parser.add_argument("--y", type=float, default=0.6)
    parser.add_argument("--wheel-base", type=float, default=WHEEL_BASE, help="wheel base usado pela odometria")
    args = parser.parse_args()
    if args.experiment == "goal":
        run_goal(args.view, (args.x, args.y))
    elif args.experiment == "gains":
        run_gains(args.view)
    elif args.experiment == "calibrate":
        run_calibrate(args.view)
    else:
        run_square(args.view, args.wheel_base)
