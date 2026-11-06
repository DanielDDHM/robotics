"""Estimadores de pose: odometria pura, giroscopio e fusao dos dois.

Todos integram a posicao do mesmo jeito (distancia vinda dos encoders).
O que muda e de onde vem o giro (d_theta) a cada passo.
"""

import math

import numpy as np
from controllers import normalize_angle

STRAIGHT_THRESHOLD = 0.15  # rad/s: abaixo disso, segundo as rodas, o robo esta indo reto ou parado


class PoseEstimator:
    name = "base"

    def __init__(self, wheel_radius: float, wheel_base: float, ticks_per_rev: int):
        self.meters_per_tick = 2 * math.pi * wheel_radius / ticks_per_rev
        self.wheel_base = wheel_base
        self.x = self.y = self.theta = 0.0
        self.bias = 0.0  # estimativa do bias do gyro (so os estimadores que usam gyro mexem nisso)

    def update(self, d_ticks_left: int, d_ticks_right: int, gyro_z: float, dt: float) -> None:
        d_left = d_ticks_left * self.meters_per_tick
        d_right = d_ticks_right * self.meters_per_tick
        d_center = (d_left + d_right) / 2
        d_theta_odom = (d_right - d_left) / self.wheel_base

        d_theta = self.heading_increment(d_theta_odom, gyro_z, dt)

        mid_theta = self.theta + d_theta / 2
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta = normalize_angle(self.theta + d_theta)

    def heading_increment(self, d_theta_odom: float, gyro_z: float, dt: float) -> float:
        raise NotImplementedError


class WheelOdometry(PoseEstimator):
    """Licao 01: o giro vem so das rodas."""

    name = "odometria pura"

    def heading_increment(self, d_theta_odom, gyro_z, dt):
        return d_theta_odom


class GyroOdometry(PoseEstimator):
    """O giro vem do giroscopio. O bias e medido uma vez, com o robo parado na largada.

    A calibracao e por tempo, como no boot de um robo real: nao da para usar o encoder
    para saber se esta parado, porque o robo "assenta" no chao e a roda mexe um tick.
    """

    name = "gyro (bias so na largada)"

    def __init__(self, *args, calibration_time: float = 1.5):
        super().__init__(*args)
        self.calibration_time = calibration_time
        self.elapsed = 0.0
        self._samples = []

    @property
    def calibrating(self) -> bool:
        return self.elapsed < self.calibration_time

    def heading_increment(self, d_theta_odom, gyro_z, dt):
        if self.calibrating:
            # Parado: tudo o que o gyro mede e bias. A media de muitas leituras tira o ruido.
            self._samples.append(gyro_z)
            self.bias = float(np.mean(self._samples))
        self.elapsed += dt
        return (gyro_z - self.bias) * dt


class ComplementaryOdometry(GyroOdometry):
    """Gyro + correcao continua do bias usando as rodas.

    Quando as rodas dizem que o robo esta indo reto, a diferenca entre o gyro e as
    rodas e quase toda bias. Um filtro passa-baixa (ganho fixo) acompanha esse valor.
    Nos giros as rodas nao sao confiaveis (scrubbing), entao a correcao desliga.
    """

    name = "filtro complementar"

    def __init__(self, *args, gain: float = 0.003):
        super().__init__(*args)
        self.gain = gain

    def heading_increment(self, d_theta_odom, gyro_z, dt):
        omega_odom = d_theta_odom / dt
        if not self.calibrating and abs(omega_odom) < STRAIGHT_THRESHOLD:
            measured_bias = gyro_z - omega_odom
            self.bias += self.gain * (measured_bias - self.bias)
        return super().heading_increment(d_theta_odom, gyro_z, dt)


class KalmanOdometry(PoseEstimator):
    """Filtro de Kalman estendido (EKF) com estado [theta, bias do gyro, escala das rodas].

    Predicao (gyro):  theta <- theta + (gyro - bias) * dt
                      bias e escala ficam iguais, mas a incerteza deles cresce devagar
    Medicao (rodas):  omega_rodas = escala * (gyro - bias) + ruido

    A escala e o erro sistematico da licao 02 (scrubbing): as rodas "acham" que o robo
    girou ~2% a mais. Com ela no estado, o filtro aprende sozinho:
      - indo reto (giro real ~0), a diferenca entre gyro e rodas revela o BIAS
      - girando, a razao entre rodas e gyro revela a ESCALA
    Ou seja, ele calibra o wheel base em movimento, sem fita no chao.
    """

    name = "filtro de Kalman (EKF)"

    def __init__(self, *args, gyro_noise: float = 0.006, bias_walk: float = 0.0005, scale_walk: float = 0.0005,
                 wheel_noise: float = 0.08, initial_bias_std: float = 0.1, initial_scale_std: float = 0.05):
        super().__init__(*args)
        self.gyro_noise = gyro_noise
        self.bias_walk = bias_walk
        self.scale_walk = scale_walk
        self.wheel_noise = wheel_noise
        self.scale = 1.0
        # Sabe o angulo inicial (0) com certeza; bias e escala, so mais ou menos
        self.P = np.diag([0.0, initial_bias_std**2, initial_scale_std**2])

    @property
    def theta_std(self) -> float:
        return math.sqrt(self.P[0, 0])

    @property
    def effective_wheel_base(self) -> float:
        return self.wheel_base * self.scale

    def heading_increment(self, d_theta_odom, gyro_z, dt):
        # 1. Predicao: integra o gyro e aumenta a incerteza
        d_theta = (gyro_z - self.bias) * dt
        F = np.array([[1.0, -dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        Q = np.diag([(self.gyro_noise * dt) ** 2, self.bias_walk**2 * dt, self.scale_walk**2 * dt])
        self.P = F @ self.P @ F.T + Q

        # 2. Atualizacao: compara a rotacao medida pelas rodas com a prevista pelo gyro
        omega_gyro = gyro_z - self.bias
        omega_odom = d_theta_odom / dt
        innovation = omega_odom - self.scale * omega_gyro
        # Jacobiano da medicao em relacao a [theta, bias, escala]: e aqui que o "estendido" entra
        H = np.array([0.0, -self.scale, omega_gyro])
        R = self.wheel_noise**2 + (self.scale * self.gyro_noise) ** 2
        S = H @ self.P @ H + R
        K = self.P @ H / S  # ganho de Kalman: quanto acreditar na medicao, por componente do estado
        correction = K * innovation
        self.bias += correction[1]
        self.scale += correction[2]
        self.P = (np.eye(3) - np.outer(K, H)) @ self.P

        return d_theta + correction[0]
