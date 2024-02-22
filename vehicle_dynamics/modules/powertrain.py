"""
Vehicle Dynamic Model - Powertrain Class

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
from vehicle_dynamics.structures.StaticParameters import StaticParameters
from vehicle_dynamics.structures.CurrentStates import CurrentStates
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre

from vehicle_dynamics.utils.plot_function import plot_function

from vehicle_dynamics.modules.LocalLogger import LocalLogger

from scipy.interpolate import interp1d

import numpy as np

class Powertrain(object):
    """
        Powertrain is a class calculates the current Torque delivered by the engine to the wheels.
        For that is necessary select the proper gear and find the torque 
        converter multiplicator.
        gear_change() function uses engine angular velocity and car linear speed (vx)
        torque_converter() multiplication is based on input and output rotation ratio.

        Required current_state from static_parameters:
            1.engine_w_table,
            2.torque_max_table,
            3.gear_ratio,
            4.diff,
            5.diff_ni,
            6.transmition_ni,
            7.gear_selection,
            8.engine_inertia,
            9.axel_inertia,
            10.gearbox_inertia,
            11.shaft_inertia,
            12.wheel_inertia,
            13.max_brake_torque,
            14.brake_bias,

        Required Arguments:
            1. throttle
            2. brake
            3. acc_x
            4.gear[t-1]
            5. vx

        Returns: (current_state)
            1. Engine_w
            2. Gear
            3. Torque of the engine on the wheels
            4. prev_gear
            5. current_gear

    """

    def __init__(self, static_parameters: StaticParameters, logger: LocalLogger):
        super(Powertrain, self).__init__()
        self.logger = logger
        self.torque_interpolation = interp1d(static_parameters.powertrain.engine.w_table, static_parameters.powertrain.engine.torque_max)
        self.MIN_GEAR_CHANGE_INTERVAL = static_parameters.powertrain.gearbox.MIN_GEAR_CHANGE_INTERVAL
        self.CONVERTER_SYNC_TIME = static_parameters.powertrain.torque_converter.CONVERTER_SYNC_TIME
        self.current_sync = 0
        self.current_grace_period = 0
        self.static_parameters = static_parameters
        self.μ_funct = interp1d(static_parameters.powertrain.torque_converter.speed_ratio,
                                static_parameters.powertrain.torque_converter.ratio)
        self.k_in_funct = interp1d(np.array(np.linspace(0., 1., len(static_parameters.powertrain.torque_converter.factor))), np.array(static_parameters.powertrain.torque_converter.factor))
        self.previous_throtle = 0

    def gear_change_rpm(self, current_state: CurrentStates, throttle: float):
        if self.current_grace_period > 0:
            self.current_grace_period -= 1
            return False

        # Gearbox up or down shifting
        if current_state.engine_w > self.static_parameters.powertrain.gearbox.gear_max_rpm[round(throttle * 10)][current_state.gear]:
            if current_state.gear + 1 >= self.static_parameters.powertrain.gearbox.gear_ratio.size:
                return False
            current_state.gear = current_state.gear + 1
            self.current_grace_period = self.MIN_GEAR_CHANGE_INTERVAL
            return True

        elif current_state.engine_w < self.static_parameters.powertrain.gearbox.gear_min_rpm[round(throttle * 10)][current_state.gear]:
            if current_state.gear - 1 < 1:
                return False
            self.current_grace_period = self.MIN_GEAR_CHANGE_INTERVAL
            current_state.gear = current_state.gear - 1
            return True        

    def powertrain(self, current_state: CurrentStates, throttle: float, brake: float):
        # Update Converter engine_w
        # Based on page 3 from Generating Proper Integrated Dynamic Models for Vehicle Mobility (Loucas S. Louca)
        # Based on the whell velocity and engine engine_w the torque on the

        if self.gear_change_rpm(current_state, throttle):
            self.current_sync = self.CONVERTER_SYNC_TIME

        if self.current_sync > 0:
            torque_converter_out = 0
            self.current_sync -= 1
            throttle = 0.2

        # Calculate torque provided by the engine based on the engine engine_w
        torque_available = self.torque_interpolation(current_state.engine_w)
        
        if self.previous_throtle - throttle != 0 :
            self.current_grace_period += 2
            self.previous_throtle = throttle
        else:
            self.previous_throtle = throttle
        
        gas_pedal = throttle

        # checking for idle state
        if current_state.engine_w < self.static_parameters.powertrain.engine.idle_rpm:
            #throttle = 1
            current_state.engine_w = self.static_parameters.powertrain.engine.idle_rpm

        # find the torque delivered by the engine
        engine_torque = (throttle * torque_available)

        final_ratio = self.static_parameters.powertrain.gearbox.gear_ratio[current_state.gear] * self.static_parameters.powertrain.differential.ratio

        turbine_w = final_ratio * np.mean(current_state.wheel_w_vel)

        torque_converter_ratio = turbine_w / current_state.engine_w

        if torque_converter_ratio >= self.static_parameters.powertrain.torque_converter.lock_up_ratio:
            # Clutch Mode
            current_state.engine_w = turbine_w
            torque_converter_out = engine_torque
        else:
            torque_converter_ratio = 0 if torque_converter_ratio < 0 else torque_converter_ratio
            # Torque Converter
            k_in = self.k_in_funct(torque_converter_ratio)
            μ = self.μ_funct(torque_converter_ratio)

            torque_converter_in = k_in * (current_state.engine_w ** 2)
            torque_converter_out = μ * torque_converter_in

            engine_wdot = (engine_torque - torque_converter_in) / self.static_parameters.powertrain.engine.inertia
            current_state.engine_w = (current_state.engine_w + engine_wdot * self.static_parameters.time_step)

        # Gillespie equation 2-7
        traction_torque = torque_converter_out * final_ratio * self.static_parameters.powertrain.gearbox.efficiency - self.static_parameters.powertrain.gearbox.inertia * final_ratio ** 2 * np.mean(current_state.x_rr.pho_r_2dot) -\
            self.static_parameters.powertrain.differential.driveshaft_inertia * self.static_parameters.powertrain.differential.ratio ** 2 * np.mean(current_state.x_rr.pho_r_2dot)

        wheel_torque = traction_torque * self.static_parameters.powertrain.bias
        if brake > 0:
            wheel_torque = np.zeros(4)

        # --------------------Break Torque -------------------------
        brake_torque = brake * self.static_parameters.brake.max_braking_torque * self.static_parameters.brake.brake_bias

        # -------------------- Total Torque -------------------------
        if np.mean((wheel_torque - brake_torque)) <= 0 and current_state.x_a.vx <= 1e-6:
            current_state.powertrain_net_torque = np.zeros(4)
            self.logger.debug(f"case 1")

        elif current_state.x_a.vx <= 1e-6 and brake > 0:
            # IDLE state
            current_state.powertrain_net_torque = np.zeros(4)
            self.logger.debug(f"case 2")

        else:
            current_state.powertrain_net_torque = wheel_torque - brake_torque 

        # Unecessary but for code safety
        if current_state.engine_w > self.static_parameters.powertrain.engine.maximum_rpm:
            current_state.engine_w = self.static_parameters.powertrain.engine.maximum_rpm

        return current_state

