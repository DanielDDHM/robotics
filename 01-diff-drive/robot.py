"""Camada de "hardware" simulado.

A ideia e que o seu codigo de controle so enxergue o que um robo real enxergaria:
contagem de ticks dos encoders, leitura do giroscopio e um comando de torque por
motor. A pose real (ground truth) existe apenas para voce comparar com a sua odometria.
"""

from pathlib import Path

import mujoco
import numpy as np

MODEL_PATH = Path(__file__).with_name("robot.xml")

WHEEL_RADIUS = 0.035  # m
WHEEL_BASE = 0.18  # m, distancia entre as rodas
TICKS_PER_REV = 1000  # resolucao do encoder
CONTROL_HZ = 100  # frequencia do loop de controle

# Giroscopio MEMS barato (ordem de grandeza de um MPU6050), eixo z, em rad/s
GYRO_NOISE = 0.006  # ruido branco por leitura (~0.35 graus/s)
GYRO_INITIAL_BIAS = 0.035  # offset de fabrica (~2 graus/s), diferente a cada vez que liga
GYRO_WARMUP_DRIFT = 0.007  # quanto o bias muda enquanto o chip aquece (~0.4 graus/s)
GYRO_WARMUP_TAU = 60.0  # s, constante de tempo do aquecimento
GYRO_RANDOM_WALK = 0.0004  # deriva aleatoria do bias, rad/s por raiz de segundo


class SimRobot:
    def __init__(self, seed: int = 0):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)
        self.substeps = round(1.0 / (CONTROL_HZ * self.model.opt.timestep))
        self._left = self.model.joint("left").qposadr[0]
        self._right = self.model.joint("right").qposadr[0]
        # Motores reais nunca sao identicos: um lado rende um pouco menos
        self.motor_gain = np.array([1.0, 0.93])
        # O IMU usa outro gerador para nao mudar o ruido dos motores das licoes anteriores
        self.imu_rng = np.random.default_rng(seed + 1000)
        self._gyro_bias0 = GYRO_INITIAL_BIAS * self.imu_rng.choice([-1.0, 1.0])
        self._gyro_walk = 0.0
        mujoco.mj_forward(self.model, self.data)

    @property
    def dt(self) -> float:
        return 1.0 / CONTROL_HZ

    @property
    def time(self) -> float:
        return self.data.time

    def read_encoders(self) -> tuple[int, int]:
        """Ticks acumulados (esquerda, direita). Quantizados, como num encoder real."""
        q = self.data.qpos
        to_ticks = TICKS_PER_REV / (2 * np.pi)
        return int(np.floor(q[self._left] * to_ticks)), int(np.floor(q[self._right] * to_ticks))

    def set_torque(self, left: float, right: float) -> None:
        cmd = np.array([left, right]) * self.motor_gain
        cmd += self.rng.normal(0.0, 0.002, size=2)  # ruido eletrico
        self.data.ctrl[:] = cmd  # o MuJoCo satura em ctrlrange

    def read_gyro(self) -> float:
        """Velocidade de giro (yaw) em rad/s, com bias e ruido como um MEMS real."""
        ideal = self.data.sensor("gyro").data[2]
        return float(ideal + self.gyro_bias() + self.imu_rng.normal(0.0, GYRO_NOISE))

    def step(self) -> None:
        """Avanca um periodo de controle (1/CONTROL_HZ segundos)."""
        mujoco.mj_step(self.model, self.data, nstep=self.substeps)
        self._gyro_walk += self.imu_rng.normal(0.0, GYRO_RANDOM_WALK * np.sqrt(self.dt))

    def gyro_bias(self) -> float:
        """Bias real do giroscopio. Como a pose real, so serve para avaliar estimadores."""
        warmup = GYRO_WARMUP_DRIFT * (1 - np.exp(-self.data.time / GYRO_WARMUP_TAU))
        return float(self._gyro_bias0 + warmup + self._gyro_walk)

    def ground_truth_pose(self) -> tuple[float, float, float]:
        """(x, y, theta) reais. Num robo de verdade voce NAO tem isso."""
        x, y = self.data.qpos[0:2]
        w, qx, qy, qz = self.data.qpos[3:7]
        theta = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
        return float(x), float(y), float(theta)
