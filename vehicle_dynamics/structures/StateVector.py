"""
Vehicle Dynamic Model - StateVector Structure

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
import numpy as np


class StateVector(object):
    def __init__(self, x=0., y=0., z=0., roll=0., pitch=0., yaw=0., vx=0., vy=0., vz=0., wx=0., wy=0., wz=0., acc_x = 0, acc_y=0, acc_z=0):

        # Position and Euler angles
        self._x = x
        self._y = y 
        self._z = z 
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.wx = wx 
        self.wy = wy 
        self.wz = wz
        self.acc_x = acc_x
        self.acc_y = acc_y
        self.acc_z = acc_z
        self.wx_dot = 0
        self.wy_dot = 0
        self.wz_dot = 0

    def __array__(self) -> np.ndarray:
        return np.array([self._x,
                         self._y,
                         self._z,
                         self.roll,
                         self.pitch,
                         self.yaw,
                         self.vx,
                         self.vy,
                         self.vz,
                         self.wx,
                         self.wy,
                         self.wz,
                         self.acc_x,
                         self.acc_y,
                         self.acc_z,
                         self.wx_dot,
                         self.wy_dot,
                         self.wz_dot])

    def __repr__(self):
        return f"StateVector with x {self.x} y {self.y} z {self.z} roll {self.roll} pitch {self.pitch} yaw {self.yaw} vx {self.vx} vy {self.vy} vz {self.vz} wx {self.wx} wy {self.wy} wz {self.wz} acc_x {self.acc_x} acc_y {self.acc_y} acc_z {self.acc_z} wx_dot {self.wx_dot} wy_dot {self.wy_dot} wz_dot {self.wz_dot}"
        pass

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @x.setter
    def x(self, value):
        if np.isnan(value).any():
            raise ValueError("x_a was set as NaN")
        self._x = value    

    @y.setter
    def y(self, value):
        if np.isnan(value).any():
            raise ValueError("x_a was set as NaN")
        self._y = value

    @z.setter
    def z(self, value):
        if np.isnan(value).any():
            raise ValueError("x_a was set as NaN")
        self._z = value

    @property
    def roll(self):
        return self._roll

    @property
    def pitch(self):
        return self._pitch

    @property
    def yaw(self):
        return self._yaw

    @roll.setter
    def roll(self, value):
        if np.isnan(value).any():
            raise ValueError("x_a was set as NaN")
        self._roll = value
        self.sin_roll = np.sin(value)
        self.cos_roll = np.cos(value)

    @pitch.setter
    def pitch(self, value):
        if np.isnan(value).any():
            raise ValueError("x_a was set as NaN")
        self._pitch = value
        self.sin_pitch = np.sin(value)
        self.cos_pitch = np.cos(value)

    @yaw.setter
    def yaw(self, value):
        if np.isnan(value).any():
            raise ValueError("x_a was set as NaN")
        self._yaw = value
        self.sin_yaw = np.sin(value)
        self.cos_yaw = np.cos(value)
