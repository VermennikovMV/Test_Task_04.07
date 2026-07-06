import math

import numpy as np


class VioModel:
    def __init__(self, dt, sigma_a, sigma_w, sigma_n, seed):
        self.dt = dt              # шаг модели, с (частота отправки фиксирована)
        self.sigma_a = sigma_a    # м/с2   - остаток ошибки акселерометра
        self.sigma_w = sigma_w    # град/с - остаток ошибки гироскопа (курс)
        self.sigma_n = sigma_n    # м      - дрожь измерений камеры (не копится)
        self.rng = np.random.default_rng(seed)   # seed = воспроизводимость
        self.tau_v = 60.0         # с - затухание ложной скорости (камера видит скорость кадр-к-кадру, v_err ограничена)
        self.v_err = np.zeros(3)
        self.p_err = np.zeros(3)
        self.psi_err = 0.0

    def tick(self, p_true, yaw_true):
        a_ost = self.rng.normal(0, self.sigma_a, 3)             # строка 1: остаток акселерометра
        self.v_err += a_ost * self.dt                           # строка 2: интегрируем ускорение от акселя и получаем скорость
        self.v_err *= 1.0 - self.dt / self.tau_v                # строка 2а: затухание накопившегося дрейфа скорости
        self.p_err += self.v_err * self.dt                      # строка 3: интегрируем скорость и получаем позицию

        w_ost = math.radians(self.rng.normal(0, self.sigma_w))  # строка 4: остаток гироскопа
        self.psi_err += w_ost * self.dt                         # строка 5: интегрируем угловые скорости и получает угол

        c, s = math.cos(self.psi_err), math.sin(self.psi_err)
        p = np.array([c * p_true[0] - s * p_true[1],            # строка 6: поворот lat/lon относительно севера VIO
                      s * p_true[0] + c * p_true[1],            
                      p_true[2]])
        p = p + self.p_err + self.rng.normal(0, self.sigma_n, 3) # плюс дрожь камеры
        yaw = yaw_true + self.psi_err                            # строка 7: курс врёт тем же углом, как и позиция в строке 6
        return p, yaw
