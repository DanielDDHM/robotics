"""Navegacao ponto a ponto (go-to-goal) para robo diferencial."""

import math

from controllers import normalize_angle


def unicycle_to_wheels(v: float, omega: float, wheel_base: float, max_wheel_speed: float) -> tuple[float, float]:
    """Cinematica inversa: (v, omega) do robo -> velocidade de cada roda.

    Se alguma roda passar do limite, escala as duas pela mesma proporcao.
    Isso preserva a curvatura (a razao omega/v), so deixa o movimento mais lento.
    Cortar cada roda separadamente mudaria a curva que o robo faz.
    """
    v_left = v - omega * wheel_base / 2
    v_right = v + omega * wheel_base / 2
    scale = max(1.0, abs(v_left) / max_wheel_speed, abs(v_right) / max_wheel_speed)
    return v_left / scale, v_right / scale


class GoToGoal:
    """Controlador proporcional em coordenadas polares.

    rho   = distancia ate o alvo
    alpha = angulo do alvo visto pelo robo (0 = alvo bem na frente)

    omega = k_alpha * alpha              gira para apontar ao alvo
    v     = min(v_max, k_rho * rho)      desacelera ao chegar perto
            * max(0, cos(alpha))         nao anda enquanto o alvo estiver de lado ou atras
    """

    def __init__(self, k_rho: float = 1.0, k_alpha: float = 4.0, max_speed: float = 0.3, tolerance: float = 0.02):
        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.max_speed = max_speed
        self.tolerance = tolerance

    def command(self, pose: tuple[float, float, float], goal: tuple[float, float]) -> tuple[float, float, bool]:
        """Retorna (v, omega, chegou)."""
        x, y, theta = pose
        dx, dy = goal[0] - x, goal[1] - y
        rho = math.hypot(dx, dy)
        if rho < self.tolerance:
            return 0.0, 0.0, True

        alpha = normalize_angle(math.atan2(dy, dx) - theta)
        v = min(self.max_speed, self.k_rho * rho) * max(0.0, math.cos(alpha))
        omega = self.k_alpha * alpha
        return v, omega, False
