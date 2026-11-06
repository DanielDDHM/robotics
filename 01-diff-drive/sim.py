"""Experimentos da licao 01.

Uso:
  python sim.py openloop  # sem controle nenhum (roda antes de implementar qualquer TODO)
  python sim.py pid       # resposta ao degrau do PID de velocidade das rodas
  python sim.py compare   # compara P, PI e PID no mesmo degrau
  python sim.py square    # anda em quadrado e compara odometria com a pose real
  mjpython sim.py square --view   # mesmo experimento com visualizacao 3D
"""

import argparse
import math
import time

import matplotlib.pyplot as plt
import numpy as np

from controllers import SPEED_GAINS, Odometry, WheelSpeedController
from robot import CONTROL_HZ, TICKS_PER_REV, WHEEL_BASE, WHEEL_RADIUS, SimRobot

KP, KI, KD = SPEED_GAINS


def make_viewer(robot: SimRobot, enabled: bool):
    if not enabled:
        return None
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(robot.model, robot.data)
    viewer.cam.distance, viewer.cam.elevation = 2.5, -50
    viewer.cam.lookat[:] = (0.5, 0.5, 0)
    return viewer


def tick(robot: SimRobot, viewer) -> bool:
    robot.step()
    if viewer is None:
        return True
    viewer.sync()
    time.sleep(robot.dt)  # aproximadamente tempo real
    return viewer.is_running()


def run_openloop(view: bool) -> None:
    """Malha aberta: o mesmo torque nos dois motores, sem nenhum controle. Nao precisa de TODO."""
    robot = SimRobot()
    viewer = make_viewer(robot, view)
    path = []
    for _ in range(6 * CONTROL_HZ):
        robot.set_torque(0.02, 0.02)
        path.append(robot.ground_truth_pose())
        if not tick(robot, viewer):
            break

    path = np.array(path)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(path[:, 0], path[:, 1], label="trajetoria real")
    ax.axhline(0, color="k", ls=":", alpha=0.4, label="linha reta desejada")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_title("Malha aberta: mesmo torque nos dois motores")
    fig.tight_layout()
    fig.savefig("openloop.png", dpi=120)
    print("Grafico salvo em openloop.png")
    x, y, th = path[-1]
    print(f"Pose final: x={x:+.3f} y={y:+.3f} th={math.degrees(th):+.1f} graus (queria y=0, th=0)")


def step_response(gains, view: bool = False) -> np.ndarray:
    """Degrau: 0 -> 0.3 m/s em t=0.5s, depois 0.3 -> 0.1 m/s em t=2.5s.

    Retorna colunas (t, alvo, v_esq, v_dir, torque_esq, torque_dir).
    """
    robot = SimRobot()
    ctrl = WheelSpeedController(robot, gains)
    viewer = make_viewer(robot, view)
    log = []
    for _ in range(4 * CONTROL_HZ):
        t = robot.time
        target = 0.0 if t < 0.5 else 0.3 if t < 2.5 else 0.1
        ctrl.update(target, target)
        log.append((t, target, *ctrl.speeds, *robot.data.ctrl))
        if not tick(robot, viewer):
            break
    return np.array(log).T


def run_compare(view: bool) -> None:
    """Mesmo degrau com P, PI e PID, para ver o papel de cada termo."""
    configs = [
        ("So P (kp=1.5)", (1.5, 0.0, 0.0)),
        ("PI (kp=1.5, ki=8)", (1.5, 8.0, 0.0)),
        ("PI com ki exagerado (ki=60)", (1.5, 60.0, 0.0)),
        ("PID (kd=0.02)", (1.5, 8.0, 0.02)),
    ]
    fig, axes = plt.subplots(len(configs), 2, sharex=True, figsize=(12, 10))
    for (name, gains), (ax_v, ax_u) in zip(configs, axes):
        t, target, vl, vr, ul, ur = step_response(gains)
        ax_v.plot(t, target, "k--", label="alvo")
        ax_v.plot(t, vl, label="roda esq.")
        ax_v.plot(t, vr, label="roda dir.")
        ax_v.set_ylabel("m/s")
        ax_v.set_ylim(-0.05, 0.45)
        ax_v.set_title(name)
        ax_u.plot(t, ul, label="esq.")
        ax_u.plot(t, ur, label="dir.")
        ax_u.set_ylabel("torque (N.m)")
        ax_u.set_title(f"{name}: torque")
        final = slice(-CONTROL_HZ // 2, None)
        overshoot = (max(vl[(t > 0.5) & (t < 2.5)]) - 0.3) / 0.3 * 100
        print(f"{name:30s} erro final esq={np.mean(target[final] - vl[final]):+.4f} "
              f"dir={np.mean(target[final] - vr[final]):+.4f} m/s  overshoot={overshoot:.0f}%")
    axes[0, 0].legend(loc="upper right")
    axes[-1, 0].set_xlabel("tempo (s)")
    axes[-1, 1].set_xlabel("tempo (s)")
    fig.tight_layout()
    fig.savefig("compare.png", dpi=110)
    print("Grafico salvo em compare.png")


def run_pid(view: bool) -> None:
    t, target, vl, vr, ul, ur = step_response((KP, KI, KD), view)
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    ax1.plot(t, target, "k--", label="alvo")
    ax1.plot(t, vl, label="roda esq.")
    ax1.plot(t, vr, label="roda dir.")
    ax1.set_ylabel("velocidade (m/s)")
    ax1.legend()
    ax1.set_title(f"PID de velocidade: kp={KP} ki={KI} kd={KD}")
    ax2.plot(t, ul, label="torque esq.")
    ax2.plot(t, ur, label="torque dir.")
    ax2.set_ylabel("torque (N.m)")
    ax2.set_xlabel("tempo (s)")
    ax2.legend()
    fig.tight_layout()
    fig.savefig("pid.png", dpi=120)
    print("Grafico salvo em pid.png")
    final = slice(-CONTROL_HZ // 2, None)
    print(f"Erro medio no ultimo 0.5s: esq={np.mean(target[final] - vl[final]):+.4f} "
          f"dir={np.mean(target[final] - vr[final]):+.4f} m/s")


def square_plan():
    """Lista de (duracao_s, v_esq, v_dir): 4 lados de 1 m com giros de 90 graus no lugar."""
    v, turn_rate = 0.3, 1.0  # m/s, rad/s
    v_turn = turn_rate * WHEEL_BASE / 2
    side = (1.0 / v, v, v)
    turn = ((math.pi / 2) / turn_rate, -v_turn, v_turn)
    pause = (0.5, 0.0, 0.0)
    return [side, pause, turn, pause] * 4


def run_square(view: bool) -> None:
    robot = SimRobot()
    ctrl = WheelSpeedController(robot)
    odom = Odometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REV)
    viewer = make_viewer(robot, view)
    truth, est = [], []
    running = True
    for duration, vl, vr in square_plan():
        for _ in range(round(duration * CONTROL_HZ)):
            ctrl.update(vl, vr)
            odom.update(*ctrl.d_ticks)
            truth.append(robot.ground_truth_pose())
            est.append((odom.x, odom.y, odom.theta))
            running = tick(robot, viewer)
            if not running:
                break
        if not running:
            break

    truth, est = np.array(truth), np.array(est)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(truth[:, 0], truth[:, 1], label="real (ground truth)")
    ax.plot(est[:, 0], est[:, 1], "--", label="odometria")
    ax.plot([0, 1, 1, 0, 0], [0, 0, 1, 1, 0], "k:", alpha=0.4, label="quadrado ideal")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_title("Odometria vs. realidade")
    fig.tight_layout()
    fig.savefig("square.png", dpi=120)
    print("Grafico salvo em square.png")
    err = np.hypot(*(truth[-1, :2] - est[-1, :2]))
    print(f"Pose final real:      x={truth[-1, 0]:+.3f} y={truth[-1, 1]:+.3f} th={math.degrees(truth[-1, 2]):+.1f} graus")
    print(f"Pose final odometria: x={est[-1, 0]:+.3f} y={est[-1, 1]:+.3f} th={math.degrees(est[-1, 2]):+.1f} graus")
    print(f"Erro de posicao da odometria: {err * 100:.1f} cm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    experiments = {"openloop": run_openloop, "pid": run_pid, "compare": run_compare, "square": run_square}
    parser.add_argument("experiment", choices=experiments)
    parser.add_argument("--view", action="store_true", help="abre o viewer 3D (use mjpython no macOS)")
    args = parser.parse_args()
    try:
        experiments[args.experiment](args.view)
    except NotImplementedError:
        raise SystemExit(
            "\nFalta implementar um TODO em controllers.py "
            "(PID.update para 'pid', PID.update e Odometry.update para 'square').\n"
            "Veja README.md. O experimento 'openloop' roda sem nada implementado."
        )
