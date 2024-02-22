"""
Vehicle Dynamic Model - Body Class

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""

from vehicle_dynamics.structures.StaticParameters import StaticParameters
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.structures.CurrentStates import CurrentStates
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre

from vehicle_dynamics.utils.plot_function import plot_function

from vehicle_dynamics.modules.LocalLogger import LocalLogger

import numpy as np
import logging
from copy import copy
import yaml


class Body():
    def __init__(self, static_parameters, logger: logging.Logger):
        self.static_parameters = static_parameters
        self.logger = logger

    def access_z_road(self, x, y):
        # TODO implement topography
        z = 0.
        return z

    def road(self, current_state):
        for k in range(4):
            # displacement.zs[k][2] = self.access_z_road(self.displacement.zs[k][0],self.displacement.zs[k][1])
            current_state.displacement.road[k] = 0.

        return current_state 

    def body(self, current_state: CurrentStates):
        """
         body is a function that calculates the current wheel loads (z)

         Required current_state from static_parameters:
             1. suspension_spring_rate
             2. suspension_damping_rate

         Required Arguments:
             1. f_zr
                 1.01 wheel_load_z
            2.displacement
                2.01 za    :  position of the connection point of the suspension with the dampner spring system
                2.02 zs    :  Z velocity of the wheel
                2.03 l_stat:  constant
                2.04 za_dot:  velocity of the chassis point
            3.vehicle_fixed2inertial_system
         Returns:
             1. f_zr.wheel_load_z

         """
        vx = current_state.x_a.vx
        vy = current_state.x_a.vy
        Ф = current_state.x_a.roll
        θ = current_state.x_a.pitch
        Ψ = current_state.x_a.yaw
        sin_Ф = current_state.x_a.sin_roll
        cos_Ф = current_state.x_a.cos_roll
        sin_θ = current_state.x_a.sin_pitch
        cos_θ = current_state.x_a.cos_pitch
        sin_Ψ = current_state.x_a.sin_yaw
        cos_Ψ = current_state.x_a.cos_yaw
        θdot = current_state.x_a.wx
        Фdot = current_state.x_a.wy
        Ψdot = current_state.x_a.wz
        θdotdot = current_state.x_a.wx_dot
        Фdotdot = current_state.x_a.wy_dot
        Ψdotdot = current_state.x_a.wz_dot
        h = self.static_parameters.body.sz
        m = self.static_parameters.body.mass

        sum_f_wheel = np.sum(current_state.x_rf.wheel_forces_transformed_force2vehicle_sys, axis=1)

        current_state = self.road(current_state)  # placeholder to consider road status

        # x-position from Vehicle CoG to the front axle [m]
        long_f = self.static_parameters.body.lf
        # x-position from Vehicle Cog to the rear axle [m]
        long_r = self.static_parameters.body.lr
        # y-position from Vehicle CoG to the left chassis point [m]
        lat_l = self.static_parameters.body.wl
        # y-position from Vehicle CoG to the right chassis point  [m]
        lat_r = self.static_parameters.body.wr

        l = long_r + long_f
        w = lat_l + lat_r


        longitudinal_distance = np.array([long_f, - long_r, long_f, - long_r])
        lateral_distance = np.array([lat_l, lat_l, - lat_r, - lat_r])

        zCG = current_state.x_a.z
        static_cg_height = current_state.reference_zCG

        displacement_suspension = zCG - static_cg_height + (-1 * longitudinal_distance * sin_θ) + (lateral_distance * sin_Ф) - current_state.displacement.road

        current_state.displacement.suspension_dot = (displacement_suspension - current_state.displacement.suspension) / self.static_parameters.time_step
        current_state.displacement.suspension = displacement_suspension

        sprung_mass = self.static_parameters.sprung_mass

        damping_force = -(self.static_parameters.suspension.damping_rate * current_state.displacement.suspension_dot)
        spring_force = -(self.static_parameters.suspension.spring_rate * displacement_suspension)

        anti_roll_bar_force = self.static_parameters.suspension.roll_bar_stiffness * sin_Ф / (2 * lateral_distance)

        suspension_force = spring_force + damping_force + anti_roll_bar_force

        current_state.suspension_force = suspension_force

        front_wheels_sum = sprung_mass[0] + sprung_mass[2]
        rear_wheels_sum = sprung_mass[1] + sprung_mass[3]

        static_force = self.static_parameters.wd * self.static_parameters.body.mass


        ξ_lon = np.array([-long_r/l, long_f/l, - long_r/l, long_f/l])
        ξ_lat = np.array([- lat_r/w, - lat_r/w, lat_l/w, lat_l/w])

        lateral_force_transfer = ξ_lat * (m * current_state.x_a.acc_y * self.static_parameters.suspension.roll_centre_height / self.static_parameters.track_width)

        longitudinal_force_transfer = ξ_lon * (m* current_state.x_a.acc_x * self.static_parameters.suspension.pitch_centre_height / self.static_parameters.wheel_base)

        current_state.f_zr.wheel_load_z = (sprung_mass+self.static_parameters.suspension.unsprung_mass) * self.static_parameters.gravity + suspension_force + longitudinal_force_transfer - lateral_force_transfer

        current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[2, :] = current_state.f_zr.wheel_load_z 


        # CHASSIS

        fy_fl = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 0]
        fy_rl = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 1]
        fy_fr = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 2]
        fy_rr = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[1, 3]

        fx_fl = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 0]
        fx_rl = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 1]
        fx_fr = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 2]
        fx_rr = current_state.x_rf.wheel_forces_transformed_force2vehicle_sys[0, 3]

        #self.logger.info(sum_f_wheel[0])

        current_state.x_a.acc_x = (sum_f_wheel[0] - 0.5 * self.static_parameters.aerodynamics_front * vx**2) / m + vy*Ψdot
        current_state.x_a.vx += current_state.x_a.acc_x * self.static_parameters.time_step
        
        if current_state.x_a.vx <= 0.0:
            current_state.x_a.acc_x = 0.0
            current_state.x_a.vx = 0.0
        

        current_state.x_a.acc_y = ((sum_f_wheel[1]) / m) + vx* Ψdot 
        current_state.x_a.acc_z = sum(suspension_force) / m

        current_state.x_a.vy += current_state.x_a.acc_y * self.static_parameters.time_step
        current_state.x_a.vz += current_state.x_a.acc_z * self.static_parameters.time_step

        vxx, vyx = vx * cos_Ψ, vy * (-sin_Ψ)
        vxy, vyy = vx * (-sin_Ψ), vy * cos_Ψ

        current_state.x_a.x += (vxx - vyx) * self.static_parameters.time_step  # (vxx + vyx)
        current_state.x_a.y += (vxy + vyy) * self.static_parameters.time_step  # (vxy + vyy)
        current_state.x_a.z += current_state.x_a.vz * self.static_parameters.time_step

        # Chassis rotation

        Ix = self.static_parameters.body.i_x_s
        Iy = self.static_parameters.body.i_y_s
        Iz = self.static_parameters.body.i_z
        h = self.static_parameters.body.sz
        l = self.static_parameters.body.wr    # halftrack

        Fx = sum_f_wheel[0]
        Fy = sum_f_wheel[1]

        #Mz = ((fy_fl + fy_fr) * self.static_parameters.body.lf - (fy_rl + fy_rr) * self.static_parameters.body.lr + (fx_rr + fx_fr - fx_rl - fx_fl) * l)
        Mz = ((fy_fl + fy_fr) * self.static_parameters.body.lf - (fy_rl + fy_rr) * self.static_parameters.body.lr + (fx_rr + fx_fr) * (self.static_parameters.body.wr) + (fx_rl + fx_fl) * (-self.static_parameters.body.wl))

        hsr = h - self.static_parameters.suspension.roll_centre_height
        hsp = h - self.static_parameters.suspension.pitch_centre_height
        current_state.x_a.wx_dot = (np.sum(lateral_distance * suspension_force) + np.sum(sprung_mass) * hsr * (current_state.x_a.acc_y + self.static_parameters.gravity * sin_Ф)) / Ix
        current_state.x_a.wy_dot = ((-1) * ((np.sum(longitudinal_distance * suspension_force) + np.sum(sprung_mass) * hsp * (current_state.x_a.acc_x - self.static_parameters.gravity * sin_θ)) / Iy))
        current_state.x_a.wz_dot = Mz / Iz   #- h * (Fx * sin_Ф + Fy * sin_θ * cos_Ф)) / (Ix * (sin_θ**2) + (cos_θ**2) * (Iy * (sin_Ф**2) + Iz * (cos_Ф**2)))

        current_state.x_a.wx += current_state.x_a.wx_dot * self.static_parameters.time_step
        current_state.x_a.wy += current_state.x_a.wy_dot * self.static_parameters.time_step
        current_state.x_a.wz += (current_state.x_a.wz_dot * self.static_parameters.time_step)

        # Angular position
        current_state.x_a.roll += (current_state.x_a.wx * self.static_parameters.time_step)
        current_state.x_a.pitch += (current_state.x_a.wy * self.static_parameters.time_step)
        current_state.x_a.yaw += (current_state.x_a.wz * self.static_parameters.time_step)

        return current_state