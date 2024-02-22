"""
Vehicle Dynamic Model - OutputStates Structure

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement

from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition
from collections import namedtuple
from copy import deepcopy, copy
import numpy as np
OutputState = namedtuple('OutputState', 'compiled_wheel_forces  delta displacement  engine_w f_zr gear powertrain_net_torque slip_x slip_y  sum_f_wheel wheel_w_vel x_a x_rf x_rr')


class OutputStates(object):
    """OutputStates Class
        makes a deep copy of the simulation parameters, and stores them in a list, any parameter is available using the .attribute[] operator"""

    def __init__(self):
        super(OutputStates, self).__init__()
        self.compiled_wheel_forces = []
        self.delta = []
        self.displacement = []
        self.engine_w = []
        self.f_zr = []
        self.gear = []
        self.get_data = []
        self.powertrain_net_torque = []
        self.slip_x = []
        self.slip_y = []
        self.sum_f_wheel = []
        self.wheel_w_vel = []
        self.x_a = []
        self.x_rf = []
        self.x_rr = []

    def __len__(self):
        return len(self.compiled_wheel_forces)

    def set_states(self, current_states):
        self.compiled_wheel_forces.append(copy(current_states.compiled_wheel_forces))
        self.delta.append(copy(current_states.delta))
        self.displacement.append(copy(current_states.displacement))
        self.engine_w.append(copy(current_states.engine_w))
        self.f_zr.append(deepcopy(current_states.f_zr))
        self.gear.append(copy(current_states.gear))
        self.powertrain_net_torque.append(copy(current_states.powertrain_net_torque))
        self.slip_x.append(copy(current_states.slip_x))
        self.slip_y.append(copy(current_states.slip_y))
        self.sum_f_wheel.append(copy(current_states.sum_f_wheel))
        self.wheel_w_vel.append(copy(current_states.wheel_w_vel))
        self.x_a.append(deepcopy(current_states.x_a))
        self.x_rf.append(deepcopy(current_states.x_rf))
        self.x_rr.append(deepcopy(current_states.x_rr))

    def __getitem__(self, items):
        return OutputState(self.compiled_wheel_forces,
                           self.delta,
                           self.displacement,
                           self.engine_w,
                           self.f_zr,
                           self.gear,
                           self.powertrain_net_torque,
                           self.slip_x,
                           self.slip_y,
                           self.sum_f_wheel,
                           self.wheel_w_vel,
                           self.x_a,
                           self.x_rf,
                           self.x_rr)

    def padding(self, value):
        self.compiled_wheel_forces.extend([self.compiled_wheel_forces[-1]] * value) 
        self.delta.extend([self.delta[-1]] * value) 
        self.displacement.extend([self.displacement[-1]] * value) 
        self.engine_w.extend([self.engine_w[-1]] * value) 
        self.f_zr.extend([self.f_zr[-1]] * value) 
        self.gear.extend([self.gear[-1]] * value) 
        self.powertrain_net_torque.extend([self.powertrain_net_torque[-1]] * value) 
        self.slip_x.extend([self.slip_x[-1]] * value) 
        self.slip_y.extend([self.slip_y[-1]] * value) 
        self.sum_f_wheel.extend([self.sum_f_wheel[-1]] * value) 
        self.wheel_w_vel.extend([self.wheel_w_vel[-1]] * value) 
        self.x_a.extend([self.x_a[-1]] * value) 
        self.x_rf.extend([self.x_rf[-1]] * value) 
        self.x_rr.extend([self.x_rr[-1]] * value) 
        pass
