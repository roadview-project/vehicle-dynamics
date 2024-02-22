"""
Vehicle Dynamic Model - Displacement Structure

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
import numpy as np


class Displacement(object):
    def __init__(self, static_suspension, suspension=np.zeros(4), suspension_dot=np.zeros(4), zr_dot=np.zeros(4), zr_2dot = np.zeros(4), road =np.zeros(4)):
        self.static_suspension = static_suspension
        self.suspension = suspension
        self.suspension_dot = suspension_dot
        self.zr_2dot = zr_2dot
        self.zr_dot = zr_dot
        self.road = road

    def __array__(self) -> np.ndarray:
        return np.array([self.static_suspension,
                         self.suspension,
                         self.suspension_dot,
                         self.zr_2dot,
                         self.zr_dot,
                         self.road])
