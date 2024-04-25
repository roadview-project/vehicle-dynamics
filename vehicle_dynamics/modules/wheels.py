from vehicle_dynamics.utils.StaticParameters import StaticParameters
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.utils.CurrentStates import CurrentStates
from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger
from vehicle_dynamics.utils.plot_function import plot_function
from copy import copy
import matplotlib.pyplot as plt
import numpy as np
import logging
import yaml 


class Wheels:
    def __init__(self, static_parameters, logger: logging.Logger):
        self.static_parameters = static_parameters
        # x-position from Vehicle CoG to the front axle [m]
        self.long_f = static_parameters.body.lf
        # x-position from Vehicle Cog to the rear axle [m]
        self.long_r = - static_parameters.body.lr
        # y-position from Vehicle CoG to the left chassis point [m]
        self.lat_l = static_parameters.body.wl
        # y-position from Vehicle CoG to the right chassis point  [m]
        self.lat_r = -static_parameters.body.wr
        pass

    def wheels(self, current_state: CurrentStates, steering_input: float):
        """
        wheels is a function that calculates the current angle
        of the frontal wheel based on the vehicle coordinate system,
        where the output delta is steering angle of the frontal wheels.

        This method calculate the wheel slip on each wheel. This is done using the relative 
        velocity between the wheel angular speed and the vehicle speed.

        Tire model calculates the wheel forces fx and fy using the Magic Formula

        Required current_state from static_parameters:
            1. steering_lock
            2. steering_ratio
            3. dynamic_radius
        Required Arguments:
            1.steering_inputtire.
            2. x_a
                2.01 vx
                2.02 vy
            3. wheel_w_vel
        Returns:
            1. delta
            2. slip_x
            3. slip_y

        """
        # Convert Steering input [-1,1] to wheel steering (delta)
        steering_angle = steering_input * self.static_parameters.steering.maximum_steering_angle 
        current_state.delta = steering_angle / self.static_parameters.steering.ratio

        # Slip Calculation
        MINIMUM_SPEED_VALUE = 15
        if (abs(self.static_parameters.tire.dynamic_radius * current_state.wheel_w_vel).all() <= MINIMUM_SPEED_VALUE) and (abs(current_state.x_a.vx) <= MINIMUM_SPEED_VALUE):
            current_state.slip_x = ((((self.static_parameters.tire.dynamic_radius * current_state.wheel_w_vel) - current_state.x_a.vx) / (MINIMUM_SPEED_VALUE)))
        else:
            # equation 11.30 Bardini
            current_state.slip_x = ((((self.static_parameters.tire.dynamic_radius * current_state.wheel_w_vel) - current_state.x_a.vx) / np.maximum(np.absolute(self.static_parameters.tire.dynamic_radius * current_state.wheel_w_vel), np.absolute(current_state.x_a.vx))))


        previous_slip_y = copy(current_state.slip_y)
        # Refenrence from VTI Y Slip angle
        if (current_state.x_a.vx <= MINIMUM_SPEED_VALUE and current_state.x_a.wz * self.lat_r <= MINIMUM_SPEED_VALUE):
            current_state.slip_y[0] = (current_state.delta - np.arctan((current_state.x_a.vy + self.long_f * current_state.x_a.wz) / MINIMUM_SPEED_VALUE))   # Front Left
            current_state.slip_y[1] = (- np.arctan((current_state.x_a.vy + self.long_r * current_state.x_a.wz) / MINIMUM_SPEED_VALUE))  # Rear Left
            current_state.slip_y[2] = (current_state.delta - np.arctan((current_state.x_a.vy + self.long_f * current_state.x_a.wz) / MINIMUM_SPEED_VALUE)) # Front Right
            current_state.slip_y[3] = (- np.arctan((current_state.x_a.vy + self.long_r * current_state.x_a.wz) / MINIMUM_SPEED_VALUE))  # Rear Right
        else:
            current_state.slip_y[0] = (current_state.delta - np.arctan((current_state.x_a.vy + self.long_f * current_state.x_a.wz) / (current_state.x_a.vx - self.lat_l * current_state.x_a.wz))) #- self.static_parameters.tire.relaxation*current_state.slip_y_rate[0]/current_state.x_a.vx   # Front Left
            current_state.slip_y[1] = (- np.arctan((current_state.x_a.vy + self.long_r * current_state.x_a.wz) / (current_state.x_a.vx - self.lat_l * current_state.x_a.wz))) #- self.static_parameters.tire.relaxation*current_state.slip_y_rate[1]/current_state.x_a.vx  # Rear Left
            current_state.slip_y[2] = (current_state.delta - np.arctan((current_state.x_a.vy + self.long_f * current_state.x_a.wz) / (current_state.x_a.vx - self.lat_r * current_state.x_a.wz))) #- self.static_parameters.tire.relaxation*current_state.slip_y_rate[2]/current_state.x_a.vx   # Front Right
            current_state.slip_y[3] = (- np.arctan((current_state.x_a.vy + self.long_r * current_state.x_a.wz) / (current_state.x_a.vx - self.lat_r * current_state.x_a.wz))) #- self.static_parameters.tire.relaxation*current_state.slip_y_rate[3]/current_state.x_a.vx # Rear Right

        current_state.slip_y_rate = (previous_slip_y - current_state.slip_y)/self.static_parameters.time_step

        # Tire Model

        current_state.x_rf.fx = current_state.f_zr.wheel_load_z * self.static_parameters.tire.longitudinal.peak_friction * np.sin(self.static_parameters.tire.longitudinal.shape_factor * np.arctan((self.static_parameters.tire.longitudinal.slip_stiffness / (self.static_parameters.tire.longitudinal.shape_factor * self.static_parameters.tire.longitudinal.peak_friction)) * current_state.slip_x))
        current_state.x_rf.fy = current_state.f_zr.wheel_load_z * self.static_parameters.tire.lateral.peak_friction * np.sin(self.static_parameters.tire.lateral.shape_factor * np.arctan((self.static_parameters.tire.lateral.cornering_coefficient / (self.static_parameters.tire.lateral.shape_factor * self.static_parameters.tire.lateral.peak_friction)) * current_state.slip_y))



        current_state.compiled_wheel_forces = np.array([current_state.x_rf.fx, current_state.x_rf.fy, current_state.f_zr.wheel_load_z])

        delta = np.array([current_state.delta, 0.0, current_state.delta, 0.0])  # FL, RL, FR, RR
        for i in range(4):
            current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[0, i] = current_state.x_rf.fx[i] * np.cos(delta[i]) - current_state.x_rf.fy[i] * np.sin(delta[i])
            current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[1, i] = current_state.x_rf.fy[i] * np.cos(delta[i]) + current_state.x_rf.fx[i] * np.sin(delta[i]) 

        final_ratio = self.static_parameters.powertrain.gearbox.gear_ratio[current_state.gear] * self.static_parameters.powertrain.differential.ratio
        current_state.x_rr.pho_r_2dot = (current_state.powertrain_net_torque - current_state.x_rf.fx * self.static_parameters.tire.dynamic_radius - self.static_parameters.tire.rolling_resistance_coefficient*current_state.f_zr.wheel_load_z) / (self.static_parameters.tire.inertia + self.static_parameters.powertrain.gearbox.inertia * final_ratio ** 2 + self.static_parameters.powertrain.differential.driveshaft_inertia * self.static_parameters.powertrain.differential.ratio ** 2)
        current_state.wheel_w_vel = current_state.wheel_w_vel + (current_state.x_rr.pho_r_2dot * self.static_parameters.time_step)  # rad/s      
        for i in range(len(current_state.wheel_w_vel)):
            if current_state.wheel_w_vel[i] <= 0.0:
                current_state.wheel_w_vel[i] = 0.0
                current_state.x_rr.pho_r_2dot[i] = 0.0

        return current_state 
