import numpy as np


class StrutForce(object):
    def __init__(self, f_za=np.zeros(4), f_za_dot=np.zeros(4), spring_force = np.zeros(4), dumper_force = np.zeros(4)):  # TODO calculate forces on the strut
        self.f_za = f_za
        self.f_za_dot = f_za_dot
        self.dumper_force = dumper_force 
        self.spring_force = spring_force 

    def __array__(self) -> np.ndarray:
        return np.array([*self.f_za.flatten(),
                         *self.f_za_dot.flatten(),
                         *self.dumper_force.flatten(),
                         *self.spring_force.flatten()])
