"""
Vehicle Dynamic Model Main Class

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""

from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement

from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.structures.CurrentStates import CurrentStates
from vehicle_dynamics.structures.StaticParameters import StaticParameters

from vehicle_dynamics.modules.LocalLogger import LocalLogger

from vehicle_dynamics.modules.powertrain import Powertrain
from vehicle_dynamics.modules.wheels import Wheels
from vehicle_dynamics.modules.body import Body


from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import yaml
import math


class VehicleDynamics(object):
    """ This Class does the grunt of calculating VehicleDynamics!!
    Calculate longitudinal and lateral dynamics with desired values 
    of brake, steer and thorttle positons.
    """

    def __init__(self, initial_state = np.zeros(15), initial_gear = 1, frequency=1000, car_parameters_path = ""):
        self.logger = LocalLogger("MainLogger").logger
        self.logger.setLevel("INFO")

        self.static_parameters = StaticParameters(car_parameters_path, frequency)

        self.logger.info("Imported YAML car self")    
        self.current_states = CurrentStates(self.static_parameters, frequency, initial_state, initial_gear, self.logger)
        self.powertrain = Powertrain(self.static_parameters, self.logger)
        self.wheels = Wheels(self.static_parameters, self.logger)
        self.body = Body(self.static_parameters, self.logger)
        self.CUT_VALUE = 0.2

    def tick(self, throttle, brake, steering_angle):
        if self.CUT_VALUE > np.any(self.current_states.slip_x):
            throttle=throttle/2
        self.current_states = self.powertrain.powertrain(self.current_states, throttle, brake)
        self.current_states = self.wheels.wheels(self.current_states, steering_angle)
        self.current_states = self.body.body(self.current_states) 

        return self.current_states
