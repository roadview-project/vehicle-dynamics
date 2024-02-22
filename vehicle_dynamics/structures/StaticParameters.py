"""
Vehicle Dynamic Model - StaticParameters Structure

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
import yaml
import numpy as np
from munch import DefaultMunch
from copy import deepcopy
import munch


class StaticParameters(object):
    
    _initialized=False

    def __init__(self, car_parameters_path, freq=1000, optimization=False):
        super(StaticParameters, self).__init__()
        with open(car_parameters_path, 'r') as file:
            param = yaml.safe_load(file)

        self.convert_YAML2Struct(param)

        self.powertrain.gearbox.gear_max_rpm *= np.pi / 30
        self.powertrain.gearbox.gear_min_rpm *= np.pi / 30
        self.powertrain.engine.w_table *= np.pi / 30
        self.powertrain.engine.minimum_rpm *= np.pi / 30
        self.powertrain.engine.maximum_rpm *= np.pi / 30
        self.powertrain.engine.idle_rpm *= np.pi / 30

        self.wheel_base = self.body.lr + self.body.lf
        self.track_width = self.body.wl + self.body.wr

        wd1 = (self.body.lr / (self.wheel_base)) * (self.body.wr / self.track_width)
        wd2 = (self.body.lf / (self.wheel_base)) * (self.body.wr / self.track_width)
        wd3 = (self.body.lr / (self.wheel_base)) * (self.body.wl / self.track_width)
        wd4 = (self.body.lf / (self.wheel_base)) * (self.body.wl / self.track_width)

        self.wd = np.array([wd1, wd2, wd3, wd4])
        self.tire.dynamic_radius = np.array(self.tire.dynamic_radius)
        self.sprung_mass = self.body.mass * self.wd - self.suspension.unsprung_mass

        self.time_step = 1. / freq

        self.aerodynamics_front = self.aerodynamics.air_drag_coefficient * self.aerodynamics.front_area * self.aerodynamics.air_density 

        self.gravity = 9.81

        if not optimization:
            self._initialized = True


    def __setattr__(self, name, value):
        if self._initialized:
            raise AttributeError("We are not in the saving attributes phase of this code, Do Better")
        else:
            object.__setattr__(self, name, value)

    def convert_YAML2Struct(self, yaml):
        data = DefaultMunch.fromDict(yaml)
        for name in dir(data):
            current_key = getattr(data, name)
            setattr(data, name, self.check_munch_key(current_key))
            setattr(self, name, getattr(data, name))

    def check_munch_key(self, current_key):
        for i in dir(current_key):
            if isinstance(getattr(current_key, i), list):
                setattr(current_key, i, np.array(getattr(current_key, i)))
            elif isinstance(getattr(current_key, i), munch.DefaultMunch):
                setattr(current_key, i, self.check_munch_key(getattr(current_key, i)))
        return current_key


def main():
    car_parameters_path = "../../Audi_r8.yaml"

    car_parameters = StaticParameters(car_parameters_path,optimization=True)

    print(car_parameters.aerodynamics.air_drag_coefficient)
    print(car_parameters.powertrain.torque_converter.efficiency_function)
    print(car_parameters.powertrain.torque_converter.efficiency_function)
    car_parameters.gravity = 1
    print(car_parameters.gravity)
    
    car_parameters._initialized = True

    car_parameters.gravity = 9.8
    print(car_parameters.gravity)

if __name__ == '__main__':
    main()
