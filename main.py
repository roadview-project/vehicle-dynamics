""" 
Main Function for the vehicle dynamics module.
@author: Maikol Funk Drechsler, Yuri Poledna

ROADVIEW Project
Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them. UK and Swiss participants in this project are supported by Innovate UK (contract no. 10045139) and the Swiss State Secretariat for Education, Research and Innovation (Contract no.22.00123) respectevely.
"""
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.utils.plot_function import plot_function
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre
from vehicle_dynamics.structures.StateVector import StateVector

from vehicle_dynamics.utils.StaticParameters import StaticParameters

from vehicle_dynamics.VehicleDynamics import VehicleDynamics

import pandas
import numpy as np
import tqdm

from collections import namedtuple

import pickle

with open("example_data/Braking.pickle","rb") as handle:
    data=pickle.load(handle)

frequency = 1000

steering = data["steering"]
throttle = data["throttle"]
brake = data["brake"]

points = len(brake)
end_time = points/frequency

time = data["time"]

manoeuvre_M8 = Manoeuvre(steering, throttle, brake, time)

output_states = OutputStates()
initial_state = StateVector(x = np.array(data["Rel_pos_x"][0]),
                            y = np.array(data["Rel_pos_y"][0]),
                            vx = np.array(data["Velocity_X"][0]),
                            yaw = np.array(data["Yaw"][0]))

car_parameters_path = "bmw_m8.yaml"

vehicle_dynamics = VehicleDynamics(initial_state = initial_state, 
                                   initial_gear = 1, 
                                   frequency = frequency, 
                                   car_parameters_path = car_parameters_path)
simulation_range = range(0, len(time))

for i in tqdm.tqdm(simulation_range):
    try:
        output_states.set_states(vehicle_dynamics.tick(*manoeuvre_M8[i]))
    except ValueError as e:
        print(e)
        output_states.padding(len(manoeuvre_M8) - len(output_states))
        plot_function(output_states, manoeuvre_M8,data,
                      xlim=(-.25,time[-1] + 0.25),
                      plot_type = "powertrain")
        exit()

plot_function(output_states, manoeuvre_M8,
              xlim=(-.25,time[-1] + 0.25),
              data=data,
              savefig=False,
              fig_save_dir="figures/", 
              static_parameters = StaticParameters(car_parameters_path, frequency),
              initial_iteraction = 0)
