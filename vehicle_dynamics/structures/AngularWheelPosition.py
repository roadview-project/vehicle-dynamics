import numpy as np


class AngularWheelPosition(object):
    def __init__(self, pho_r=np.zeros(4), pho_r_dot = np.zeros(4), pho_r_2dot = np.zeros(4)):

        # pag 272 eq.52
        # Angular position/speed of each wheel
        self.pho_r = pho_r
        self.pho_r_dot = pho_r_dot
        self.pho_r_2dot = pho_r_2dot

    def __array__(self) -> np.ndarray:
        return np.array([self.pho_r, self.pho_r_dot, self.pho_r_2dot])
