"""
Vehicle Dynamic Model - TireForces Structure

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
import numpy as np


class TireForces(object):
    def __init__(self, fx =np.zeros(4), fy =np.zeros(4), wheel_forces_transformed_force2vehicle_sys = np.zeros((3, 4), dtype=float)):        
        # Dynamic forces on the tires
        # pag 272 eq.54
        self.fx = fx
        self.fy = fy
        self.wheel_forces_transformed_force2vehicle_sys = wheel_forces_transformed_force2vehicle_sys 

    def __array__(self) -> np.ndarray:
        return np.array([*self.fx,
                         *self.fy,
                         *self.wheel_forces_transformed_force2vehicle_sys.flatten()])
