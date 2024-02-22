"""
Vehicle Dynamic Model - WheelHubForce Structure

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""

import numpy as np


class WheelHubForce(object):
    def __init__(self, f_zr_dot=np.zeros(4), wheel_load_z =np.array([2000., 3000., 2000., 3000.])):
        self.f_zr_dot = f_zr_dot
        self.wheel_load_z = wheel_load_z

    def __array__(self) -> np.ndarray:
        return np.array([self.f_zr_dot,
                         self.wheel_load_z])
