import numpy as np


class WheelHubForce(object):
    def __init__(self, f_zr_dot=np.zeros(4), wheel_load_z =np.array([2000., 3000., 2000., 3000.])):
        self.f_zr_dot = f_zr_dot
        self.wheel_load_z = wheel_load_z

    def __array__(self) -> np.ndarray:
        return np.array([self.f_zr_dot,
                         self.wheel_load_z])
