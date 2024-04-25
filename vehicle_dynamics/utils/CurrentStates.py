from vehicle_dynamics.utils.StaticParameters import StaticParameters
from vehicle_dynamics.structures.TireForces import TireForces
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.Displacement import Displacement
from vehicle_dynamics.structures.StrutForce import StrutForce
from vehicle_dynamics.structures.WheelHubForce import WheelHubForce
from vehicle_dynamics.structures.AngularWheelPosition import AngularWheelPosition

from vehicle_dynamics.utils.import_data_CM import import_data_CM
from vehicle_dynamics.utils.LocalLogger import LocalLogger


from scipy.interpolate import interp1d
from numpy.linalg import inv
import numpy as np
import yaml
import math 


class CurrentStates(object):
    """This class initialize the values of a vehicular dynamic model. """

    def __init__(self, static_parameters, freq = 1000, initial_state = np.zeros(15), initial_gear = 1, logger = 1):
        super(CurrentStates, self).__init__()
        self.delta = 0

        self.engine_w = static_parameters.powertrain.engine.idle_rpm
        self.gear = initial_gear                      # gear selector
        self.wheel_w_vel = np.zeros(4)

        # State initiate with the position, orientation and speed provided by the arguments, acc = 0; 
        if isinstance(initial_state, np.ndarray):
            self.x_a = StateVector(*initial_state)  # initial_state
        elif isinstance(initial_state, StateVector):
            self.x_a = initial_state

        self.reference_zCG = self.x_a.z

        Ф = self.x_a.roll
        θ = self.x_a.pitch
        Ψ = self.x_a.yaw
        cos = np.cos
        sin = np.sin
        sin_Ф = self.x_a.sin_roll
        cos_Ф = self.x_a.cos_roll
        sin_θ = self.x_a.sin_pitch
        cos_θ = self.x_a.cos_pitch
        sin_Ψ = self.x_a.sin_yaw
        cos_Ψ = self.x_a.cos_yaw

        # Wheel initiate stoped 
        self.x_rr = AngularWheelPosition(pho_r=np.zeros(4), pho_r_dot = np.zeros(4), pho_r_2dot =np.zeros(4))
        self.x_rf = TireForces(fx =np.zeros(4), fy = np.zeros(4), wheel_forces_transformed_force2vehicle_sys = np.ones((3, 4), dtype=float))
        self.displacement = Displacement(static_suspension=(static_parameters.body.mass * static_parameters.wd * static_parameters.gravity) / static_parameters.suspension.spring_rate, 
                                         suspension = np.zeros(4), 
                                         suspension_dot=np.zeros(4), 
                                         zr_dot=np.zeros(4), 
                                         zr_2dot=np.zeros(4))
        zCG = self.x_a.z
        static_cg_height = static_parameters.body.sz
        long_f = static_parameters.body.lf
        # x-position from Vehicle Cog to the rear axle [m]
        long_r = - static_parameters.body.lr
        # y-position from Vehicle CoG to the left chassis point [m]
        lat_l = static_parameters.body.wl
        # y-position from Vehicle CoG to the right chassis point  [m]
        lat_r = - static_parameters.body.wr

        zCG = self.x_a.z
        static_cg_height = self.reference_zCG

        displacement_FL = zCG - static_cg_height - (sin_θ * long_f) + (sin_Ф * lat_l) - self.displacement.road[0]
        displacement_RL = zCG - static_cg_height - (sin_θ * long_r) + (sin_Ф * lat_l) - self.displacement.road[1]
        displacement_FR = zCG - static_cg_height - (sin_θ * long_f) + (sin_Ф * lat_r) - self.displacement.road[2]
        displacement_RR = zCG - static_cg_height - (sin_θ * long_r) + (sin_Ф * lat_r) - self.displacement.road[3]

        displacement_suspension = np.array([displacement_FL, displacement_RL, displacement_FR, displacement_RR])

        self.displacement.suspension_dot = (displacement_suspension - self.displacement.suspension) / static_parameters.time_step
        self.displacement.suspension = displacement_suspension

        sprung_mass = static_parameters.body.mass * static_parameters.wd - static_parameters.suspension.unsprung_mass

        damping_force = (static_parameters.suspension.damping_rate * self.displacement.suspension_dot)
        spring_force = (static_parameters.suspension.spring_rate * displacement_suspension)

        anti_roll_bar_force = static_parameters.suspension.roll_bar_stiffness * sin_Ф / (2 * np.array([lat_l, lat_l, lat_r, lat_r]))

        suspension_force = spring_force + damping_force - anti_roll_bar_force

        self.f_zr = WheelHubForce(f_zr_dot=np.array([0., 0., 0., 0.]), 
                                  wheel_load_z = static_parameters.body.mass * static_parameters.gravity * static_parameters.wd)

        static_force = static_parameters.wd * static_parameters.body.mass

        lateral_force_transfer = np.array([-1, -1, 1, 1]) * (np.sum(sprung_mass) * self.x_a.acc_y * static_parameters.suspension.roll_centre_height / static_parameters.track_width)

        longitudinal_force_transfer = np.array([-1, 1, -1, 1]) * (np.sum(sprung_mass) * self.x_a.acc_x * static_parameters.suspension.pitch_centre_height / (2 * static_parameters.wheel_base))

        self.f_zr.wheel_load_z = sprung_mass * static_parameters.gravity + suspension_force + longitudinal_force_transfer + lateral_force_transfer

        self.f_za = StrutForce(f_za = static_parameters.body.mass * static_parameters.gravity * static_parameters.wd, 
                               f_za_dot = np.zeros(4), 
                               spring_force = (static_parameters.body.mass * np.array([static_parameters.body.lr * static_parameters.body.wr, static_parameters.body.lf * static_parameters.body.wr, static_parameters.body.lr * static_parameters.body.wl, static_parameters.body.lf * static_parameters.body.wl]) / ((static_parameters.body.lr + static_parameters.body.lf) * (static_parameters.body.wl + static_parameters.body.wr)) * static_parameters.gravity).reshape(-1, 1), 
                               dumper_force = np.zeros((4, 1), dtype=float))

        self.x_rf.wheel_forces_transformed_force2vehicle_sys[2, :] = self.f_zr.wheel_load_z

        self.compiled_wheel_forces = np.array([self.x_rf.fx, self.x_rf.fy, self.f_zr.wheel_load_z])

        # Creating vector for chassis method 
        self.sum_f_wheel = np.zeros(3, dtype=float)  # Sum of wheel forces

        self.slip_x = np.zeros(4)
        self.slip_y = np.zeros(4)
        self.slip_y_rate = np.zeros(4)
        

        self.powertrain_net_torque = np.zeros(4)
        self.suspension_force = suspension_force


    @property
    def delta(self):
        return self._delta

    @delta.setter
    def delta(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"delta was set as {value}")
        self._delta = value

    @property
    def engine_w(self):
        return self._engine_w

    @engine_w.setter
    def engine_w(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"engine_w was set as {value}")
        self._engine_w = value

    @property
    def gear(self):
        return self._gear

    @gear.setter
    def gear(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"gear was set as {value}")
        self._gear = value

    @property
    def wheel_w_vel(self):
        return self._wheel_w_vel

    @wheel_w_vel.setter
    def wheel_w_vel(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"wheel_w_vel was set as {value}")
        self._wheel_w_vel = value

    @property
    def x_a(self):
        return self._x_a

    @x_a.setter
    def x_a(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"x_a was set as {value}")
        self._x_a = value

    @property
    def reference_zCG(self):
        return self._reference_zCG

    @reference_zCG.setter
    def reference_zCG(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"reference_zCG was set as {value}")
        self._reference_zCG = value

    @property
    def x_rr(self):
        return self._x_rr

    @x_rr.setter
    def x_rr(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"x_rr was set as {value}")
        self._x_rr = value

    @property
    def x_rf(self):
        return self._x_rf

    @x_rf.setter
    def x_rf(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"x_rf was set as {value}")
        self._x_rf = value

    @property
    def displacement(self):
        return self._displacement

    @displacement.setter
    def displacement(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"displacement was set as {value}")
        self._displacement = value

    @property
    def f_zr(self):
        return self._f_zr

    @f_zr.setter
    def f_zr(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"f_zr was set as {value}")
        self._f_zr = value

    @property
    def f_za(self):
        return self._f_za

    @f_za.setter
    def f_za(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"f_za was set as {value}")
        self._f_za = value

    @property
    def compiled_wheel_forces(self):
        return self._compiled_wheel_forces

    @compiled_wheel_forces.setter
    def compiled_wheel_forces(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"compiled_wheel_forces was set as {value}")
        self._compiled_wheel_forces = value

    @property
    def sum_f_wheel(self):
        return self._sum_f_wheel

    @sum_f_wheel.setter
    def sum_f_wheel(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"sum_f_wheel was set as {value}")
        self._sum_f_wheel = value

    @property
    def slip_x(self):
        return self._slip_x

    @slip_x.setter
    def slip_x(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"slip_x was set as {value}")
        self._slip_x = value

    @property
    def slip_y(self):
        return self._slip_y

    @slip_y.setter
    def slip_y(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"slip_y was set as {value}")
        self._slip_y = value

    @property
    def powertrain_net_torque(self):
        return self._powertrain_net_torque

    @powertrain_net_torque.setter
    def powertrain_net_torque(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"powertrain_net_torque was set as {value}")
        self._powertrain_net_torque = value

    @property
    def suspension_force(self):
        return self._suspension_force

    @suspension_force.setter
    def suspension_force(self, value):
        if np.isnan(value).any() or np.isinf(value).any():
            raise ValueError(f"suspension_force was set as {value}")
        self._suspension_force = value


def main():
    from vehicle_dynamics.utils.LocalLogger import LocalLogger
    from vehicle_dynamics.utils.StaticParameters import StaticParameters
    test_function = CurrentStates
    function_name = test_function.__name__
    logger = LocalLogger(function_name).logger
    logger.setLevel('INFO')
    static_parameters = StaticParameters("../../Audi_r8.yaml")
    parameters = CurrentStates(static_parameters, logger=logger)
    pass


if __name__ == '__main__':
    main()
