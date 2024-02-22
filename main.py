""" 
Main Function for the vehicle dynamics module.
@author: Maikol Funk Drechsler, Yuri Poledna

ROADVIEW Project
Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them. UK and Swiss participants in this project are supported by Innovate UK (contract no. 10045139) and the Swiss State Secretariat for Education, Research and Innovation (Contract no.22.00123) respectevely.
"""
from vehicle_dynamics.structures.OutputStates import OutputStates
from vehicle_dynamics.structures.Manoeuvre import Manoeuvre
from vehicle_dynamics.structures.StateVector import StateVector
from vehicle_dynamics.structures.StaticParameters import StaticParameters

from vehicle_dynamics.utils.plot_function import plot_function

from vehicle_dynamics.VehicleDynamics import VehicleDynamics


import pandas
import numpy as np
import tqdm


frequency = 1000
end_time = 20
points = frequency * time

steering = np.linspace(0,1,points)
throttle = np.ones(points)
brake = np.zeros(points)

time = np.linspace(0, end_time, points)

manoeuvre = Manoeuvre(steering, throttle, brake, time)

output_states = OutputStates()

car_parameters_path = "bmw_m8.yaml"

vehicle_dynamics = VehicleDynamics(initial_gear = 1, 
                                   frequency = frequency, 
                                   car_parameters_path = car_parameters_path)

for i in tqdm.tqdm(range(points)):
    try:
        output_states.set_states(vehicle_dynamics.tick(*manoeuvre[i]))
    except ValueError as e:
        print(e)
        output_states.padding(len(manoeuvre) - len(output_states))
        plot_function(output_states, manoeuvre,
                      xlim=(-.25,time[-1] + 0.25),
                      plot_type = "powertrain")
        exit()

plot_function(output_states, manoeuvre, 
              xlim=(-.25,time[-1] + 0.25),
              savefig=True,
              fig_save_dir="figures/")
