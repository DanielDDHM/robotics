"""Controladores da licao 01: PID de velocidade e odometria."""

import math

from robot import TICKS_PER_REV, WHEEL_RADIUS, SimRobot

# Ganhos ajustados com o experimento "compare" (P rapido, I para zerar o erro, sem D)
SPEED_GAINS = (1.5, 8.0, 0.0)
MAX_TORQUE = 0.3


class PID:
    """Controlador PID com saturacao e anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float, out_min: float, out_max: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement

        # D: na primeira chamada nao ha erro anterior, entao a derivada e zero
        # (senao teriamos um pico enorme, o "derivative kick")
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error

        # I: calcula a integral candidata, mas so a aceita se a saida nao saturar.
        # Isso e o anti-windup: com o motor ja no torque maximo, acumular mais
        # integral nao ajuda agora e causa overshoot depois.
        integral = self.integral + error * dt
        output = self.kp * error + self.ki * integral + self.kd * derivative
        if self.out_min < output < self.out_max:
            self.integral = integral

        return max(self.out_min, min(self.out_max, output))


def normalize_angle(angle: float) -> float:
    """Leva qualquer angulo para [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Odometry:
    """Estima a pose (x, y, theta) integrando o deslocamento medido pelos encoders."""

    def __init__(self, wheel_radius: float, wheel_base: float, ticks_per_rev: int):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.ticks_per_rev = ticks_per_rev
        self.x = self.y = self.theta = 0.0

    def update(self, d_ticks_left: int, d_ticks_right: int) -> None:
        meters_per_tick = 2 * math.pi * self.wheel_radius / self.ticks_per_rev
        d_left = d_ticks_left * meters_per_tick
        d_right = d_ticks_right * meters_per_tick

        d_center = (d_left + d_right) / 2
        d_theta = (d_right - d_left) / self.wheel_base

        # Durante o intervalo o robo girou de theta ate theta + d_theta.
        # Usar o angulo medio aproxima o arco percorrido melhor que usar so o inicial.
        mid_theta = self.theta + d_theta / 2
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta = normalize_angle(self.theta + d_theta)


class WheelSpeedController:
    """Junta encoders + PID: recebe velocidade desejada de cada roda (m/s) e aplica torque."""

    def __init__(self, robot: SimRobot, gains: tuple[float, float, float] = SPEED_GAINS):
        self.robot = robot
        self.pids = [PID(*gains, -MAX_TORQUE, MAX_TORQUE) for _ in range(2)]
        self.prev_ticks = robot.read_encoders()
        self.speeds = (0.0, 0.0)
        self.d_ticks = (0, 0)

    def update(self, target_left: float, target_right: float) -> None:
        ticks = self.robot.read_encoders()
        self.d_ticks = (ticks[0] - self.prev_ticks[0], ticks[1] - self.prev_ticks[1])
        self.prev_ticks = ticks
        meters_per_tick = 2 * math.pi * WHEEL_RADIUS / TICKS_PER_REV
        dt = self.robot.dt
        self.speeds = tuple(d * meters_per_tick / dt for d in self.d_ticks)
        torques = [
            pid.update(target, speed, dt)
            for pid, target, speed in zip(self.pids, (target_left, target_right), self.speeds)
        ]
        self.robot.set_torque(*torques)
