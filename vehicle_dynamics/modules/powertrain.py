from vehicle_dynamics.utils.StaticParameters import StaticParameters
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.CurrentStates import CurrentStates
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.plot_function import plot_function
from vehicle_dynamics.utils.StaticParameters import StaticParameters

from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre

import logging
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
        traction_torque = torque_converter_out * final_ratio * self.static_parameters.powertrain.gearbox.efficiency
        
        #if traction_torque > 5000.0:
        #    traction_torque = 5000.0

        wheel_torque = traction_torque * self.static_parameters.powertrain.bias

        #if brake > 0:
        #    wheel_torque = np.zeros(4)

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


def main(static_parameters, current_state, data, logger, savefig=False):
    import yaml
    from tqdm import tqdm

    steering = data["steering"]
    throttle = data["throttle"]
    brake = data["brake"]

    time = data["time"]

    points = len(brake)

    manoeuvre = Manoeuvre(steering, throttle, brake, time)
    logger.info("loaded SimulationData")


    fl,rl,fr,rr = (data["v_wheel_fl"],
                   data["v_wheel_fr"],
                   data["v_wheel_rl"],
                   data["v_wheel_rr"])
    
    simulation_range = range(0, len(time))

    wheel_w_vel = np.array([[fl[i],rl[i],fr[i],rr[i]] for i in range(len(fl))])
    vx = data ["Velocity_X"]
    vy = data ["Velocity_Y"]
    
    yawrate = data["Yaw_Velocity"]

    powertrain = Powertrain(static_parameters, logger)
    test_function = powertrain.powertrain
    output_states = OutputStates()

    for i in tqdm(simulation_range):
        current_state.x_a.vx = vx[i]
        current_state.wheel_w_vel = wheel_w_vel[i]
        try:
            current_state = test_function(current_state, 
                                          throttle = manoeuvre[i].throttle,
                                          brake = manoeuvre[i].brake)
            output_states.set_states(current_state)
        except ValueError as e:
            print(e)
            output_states.padding(len(manoeuvre) - len(output_states))
            plot_function(output_states, manoeuvre, sync_adma, sync_zgw, savefig, plot_type="powertrain", xlim=(-.25,sync_zgw[-1].timestamp-sync_zgw[0].timestamp + 0.25))
            exit()


    plot_function(output_states, manoeuvre, data, savefig, plot_type="powertrain", xlim=(-.25,time[-1]-time[0] + 0.25))


if __name__ == '__main__':
    import pickle

    PATH_TO_DATA = "../../example_data/Braking.pickle"
    with open(PATH_TO_DATA,"rb")as handle:
        data=pickle.load(handle)

    from vehicle_dynamics.structures.StateVector import StateVector
    from vehicle_dynamics.utils.StaticParameters import StaticParameters
    static_parameters = StaticParameters("../../bmw_m8.yaml", freq = 1000)
    
    function_name = "Powertrain"
    logger = LocalLogger(function_name).logger
    logger.info("loaded current_state")
    from vehicle_dynamics.utils.StaticParameters import StaticParameters
    static_parameters = StaticParameters("../../BMW_M8.yaml")
    from vehicle_dynamics.structures.StateVector import StateVector    

    initial_state = StateVector(x = np.array( data["Rel_pos_x"][0]),
                                y = np.array(data["Rel_pos_y"][0]),
                                vx = np.array( data["Velocity_X"][0]),
                                yaw = np.array( data["Yaw"][0]))    

    current_state = CurrentStates(static_parameters, logger=logger,initial_state=initial_state)
    main(static_parameters, current_state, data, logger, False)
