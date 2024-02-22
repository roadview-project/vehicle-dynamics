"""
Vehicle Dynamic Model - Plot Function

@author:   Maikol Funk Drechsler, Yuri Poledna

Funded by the European Union (grant no. 101069576). Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or the European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them.
"""
def define_colors():
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    ROADVIEW_colors = {"ROADVIEW1": "#64798C",
                       "ROADVIEW2": "#B0C6D9",
                       "ROADVIEW3": "#F2C879",
                       "ROADVIEW4": "#F28066"}

    mcolors.get_named_colors_mapping().update(ROADVIEW_colors)
    return mcolors.LinearSegmentedColormap.from_list("", ["ROADVIEW1", "ROADVIEW2", "ROADVIEW3"])


def plot_function(output_states, manoeuvre, savefig=False, plot_type = "all", xlim=(), fig_save_dir="../../figures"):
    import matplotlib.pyplot as plt
    import numpy as np
    size = 17
    figsize = (size, size*.588)
    simulation_range = range(0, len(manoeuvre))
    throttle = manoeuvre[:].throttle
    plt.rcParams.update({'font.size': 16})

    define_colors()

    states = output_states[:].x_a
    if "body" in plot_type or plot_type == "all":

        plt.figure(figsize=figsize)
        name = " Chassis Angles"
        plt.title(name)
        var_name = "Vhcl_Roll"
        plt.plot(manoeuvre.time, [i.roll for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Roll")
        plt.plot(manoeuvre.time, [i.pitch for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Pitch")
        plt.plot(manoeuvre.time, [i.yaw -55*3.14/180 for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Yaw")    
        plt.grid()
        plt.legend()
        plt.xlim(xlim)
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (rad)")
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")  
        plt.figure(figsize=figsize)
        name = " Chassis Angles Velocity"
        plt.title(name)
        plt.plot(manoeuvre.time, [i.wx for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Roll Velocity")
        plt.plot(manoeuvre.time, [i.wy for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Pitch Velocity")
        plt.plot(manoeuvre.time, [i.wz for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Yaw Velocity")

        plt.xlabel("Time (s)")
        plt.ylabel("Angle Velocity (rad/s)")
        plt.legend()
        plt.xlim(xlim)
        plt.grid()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")   

        plt.figure(figsize=figsize)
        name = " Chassis Acceleration"
        plt.title(name)
        plt.plot(manoeuvre.time, [i.acc_x for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Acceleration X")
        plt.plot(manoeuvre.time, [i.acc_y for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Acceleration Y")
        plt.plot(manoeuvre.time, [i.acc_z for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Acceleration Z")
        plt.grid()
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (m/s²)")
        plt.legend()
        plt.xlim(xlim)
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")     
        plt.figure(figsize=figsize)
        name = " Chassis Velocity"
        plt.title(name)

        plt.plot(manoeuvre.time, [i.vx for i in output_states[:].x_a], "ROADVIEW1", linestyle="--", label="Calculated Velocity X")
        plt.plot(manoeuvre.time, [i.vy for i in output_states[:].x_a], "ROADVIEW2", linestyle="--", label="Calculated Velocity Y")
        plt.plot(manoeuvre.time, [i.vz for i in output_states[:].x_a], "ROADVIEW3", linestyle="--", label="Calculated Velocity Z")

        plt.grid()
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.xlim(xlim)
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")   

        plt.figure(figsize=figsize)
        name = " Chassis Position"
        plt.title(name)

        plt.plot([i.x for i in output_states[:].x_a], [i.y for i in output_states[:].x_a], "ROADVIEW4", linestyle="--", label="Calculated Position")

        plt.grid()
        plt.xlabel("Position X (m)")
        plt.ylabel("Position Y (m)")
        plt.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/" + name + ".png")   

    if "powertrain" in plot_type or plot_type == "all":
        gear_changes_idx =np.where(np.roll(output_states[:].gear, 1) != output_states[:].gear)[0]
        gear_changes = manoeuvre.time[gear_changes_idx]

        f, (a0, a1) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [2, 2]}, figsize=figsize, sharex=True)
        powertrain_net_torque = [np.sum(i) for i in output_states[:].powertrain_net_torque]
        a0.plot(manoeuvre.time, powertrain_net_torque, "ROADVIEW1", linestyle="-", label="Calculated")
        
        powertrain_net_torque_M8 = engine_torque_M8 + break_torque_M8
        a0.plot(vehicle_time, powertrain_net_torque_M8, "ROADVIEW2",  linestyle="--", label = "BMW M8")
        if len(gear_changes)>0:
            a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
            a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
            for j,i in enumerate(gear_changes[1:]):
                a0.axvline(i, color="ROADVIEW4", linestyle=":")
                a1.axvline(i, color="ROADVIEW4", linestyle=":") 
        a0.legend()
        a0.grid()
        a0.set_ylabel("Powertrain Net Torque (Nm)")
        a0.set_xlim(xlim)
        a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "-", label = "Gas pedal(BMW M8)")
        a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "-", label = "Brake pedal(BMW M8)")
        a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Calculated Gear Number")
        a1.legend(loc=1)
        a1.set_xlabel("Time (s)")
        a1.grid()
        a1.set_xlim(xlim)
        a1.set_ylim([-0.1, 2.3])
        plt.subplots_adjust(wspace = 0, hspace = 0)
        if savefig:
            plt.savefig(fig_save_dir + "/Powertrain Nettorque.png")

        f, (a0, a1) = plt.subplots(2, 1, gridspec_kw = {'height_ratios': [2, 2]}, figsize = figsize, sharex=True)

        a0.plot(manoeuvre.time, (np.array(output_states[:].engine_w) * 30 / np.pi), "ROADVIEW1", linestyle = "--", label = "Calculated")
        a0.set_ylabel("Engine Rotation (RPM)")

        a0.legend()
        a0.grid()
        a0.set_xlim(xlim)
        
        a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Gear Number (Calculated)")
        a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "-", label = "Gas pedal (BMW M8)")
        a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "-", label = "Brake pedal (BMW M8)")
        a1.set_xlim(xlim)
        a1.set_ylim([-0.1, 2.3])
        a1.grid()
        if len(gear_changes)>0:
            a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
            a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
            for j,i in enumerate(gear_changes[1:]):
                a0.axvline(i, color="ROADVIEW4", linestyle=":")
                a1.axvline(i, color="ROADVIEW4", linestyle=":") 
        a1.legend(loc =1)
        a1.set_xlabel("Time (s)")
        plt.subplots_adjust(wspace = 0, hspace = 0) 
        if savefig:
            plt.savefig(fig_save_dir + "/Powertrain RPM.png")

    if "wheels" in plot_type or plot_type == "all":


        gear_changes_idx =np.where(np.roll(output_states[:].gear, 1) != output_states[:].gear)[0]
        gear_changes = manoeuvre.time[gear_changes_idx]

        f, (a0, a1) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [3, 1]}, figsize=figsize, sharex=True)
        powertrain_net_torque = [np.sum(i) for i in output_states[:].powertrain_net_torque]
        a0.plot(manoeuvre.time, powertrain_net_torque, "ROADVIEW3", linestyle="--", label="Calculated")
        
        powertrain_net_torque_M8 = engine_torque_M8 + break_torque_M8
        a0.plot(vehicle_time, powertrain_net_torque_M8, "ROADVIEW1", label = "BMW M8")
        if len(gear_changes)>0:
            a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
            a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
            for j,i in enumerate(gear_changes[1:]):
                a0.axvline(i, color="ROADVIEW4", linestyle=":")
                a1.axvline(i, color="ROADVIEW4", linestyle=":") 
        a0.legend()
        a0.grid()
        a0.set_ylabel("Powertrain Net Torque (Nm)")
        a0.set_xlim(xlim)
        a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "--", label = "Gas pedal(BMW M8)")
        a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "--", label = "Brake pedal(BMW M8)")
        a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Calculated Gear Number")
        a1.legend(loc=1)
        a1.set_xlabel("Time (s)")
        a1.grid()
        a1.set_xlim(xlim)
        plt.subplots_adjust(wspace = 0, hspace = 0)
        if savefig:
            plt.savefig(fig_save_dir + "/Powertrain Nettorque.png")

        f, (a0, a1) = plt.subplots(2, 1, gridspec_kw = {'height_ratios': [3, 1]}, figsize = figsize, sharex=True)

        a0.plot(manoeuvre.time, (np.array(output_states[:].engine_w) * 30 / np.pi), "ROADVIEW1", linestyle = "--", label = "Calculated")
        a0.set_ylabel("Engine Rotation (RPM)")

        a0.legend()
        a0.grid()
        a0.set_xlim(xlim)
        
        a1.step(manoeuvre.time, output_states[:].gear, "ROADVIEW1", linestyle = "-", label = "Gear Number (Calculated)")
        a1.plot(manoeuvre.time, manoeuvre.throttle, "ROADVIEW3", linestyle = "-", label = "Gas pedal (BMW M8)")
        a1.plot(manoeuvre.time, manoeuvre.brake, "ROADVIEW4", linestyle = "-", label = "Brake pedal (BMW M8)")
        a1.set_xlim(xlim)
        a1.grid()
        if len(gear_changes)>0:
            a0.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment")
            a1.axvline(gear_changes[0], color="ROADVIEW4", linestyle=":", label="Gear Change Moment") 
            for j,i in enumerate(gear_changes[1:]):
                a0.axvline(i, color="ROADVIEW4", linestyle=":")
                a1.axvline(i, color="ROADVIEW4", linestyle=":") 
        a1.legend(loc =1)
        a1.set_xlabel("Time (s)")
        plt.subplots_adjust(wspace = 0, hspace = 0) 
        if savefig:
            plt.savefig(fig_save_dir + "/Powertrain RPM.png")
            

        MINIMUM_SPEED_VALUE = 10


        slip_fr = np.zeros(len(vx))
        slip_rr = np.zeros(len(vx)) 
        slip_fl = np.zeros(len(vx))
        slip_rl = np.zeros(len(vx))

        f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
        a0.plot(vehicle_time, slip_fl, "ROADVIEW1", label="Longitudinal Slip FL")
        a0.plot(manoeuvre.time, [i[0] for i in output_states[:].slip_x], "ROADVIEW1", linestyle ="--", label ="Calculated Longitudinal Slip FL")
        a0.set_xlim(xlim)
        a0.legend()
        a0.set_ylim(-0.2, 0.2)
        a0.set_xlabel("Time (s)")
        a0.set_ylabel("Wheel Slip")
        a2.set_xlabel("Time (s)")
        a2.set_ylabel("Wheel Slip")
        a2.plot(vehicle_time, slip_rl, "ROADVIEW2", label="Longitudinal Slip RL")
        a2.plot(manoeuvre.time, [i[1] for i in output_states[:].slip_x], "ROADVIEW2", linestyle ="--", label ="Calculated Longitudinal Slip RL")
        a2.set_xlim(xlim)
        a2.legend()
        a1.set_ylim(-0.2, 0.2)
        a1.set_xlabel("Time (s)")
        a1.set_ylabel("Wheel Slip")
        a1.plot(vehicle_time, slip_fr, "ROADVIEW3", label="Longitudinal Slip FR")
        a1.plot(manoeuvre.time, [i[2] for i in output_states[:].slip_x], "ROADVIEW3", linestyle ="--", label ="Calculated Longitudinal Slip FR")
        a1.set_xlim(xlim)
        a1.legend()
        a1.set_ylim(-0.2, 0.2)
        a3.set_ylim(-0.2, 0.2)
        a3.set_xlabel("Time (s)")
        a3.set_ylabel("Wheel Slip")
        a3.plot(vehicle_time, slip_rr, "ROADVIEW4", label="Longitudinal Slip RR")
        a3.plot(manoeuvre.time, [i[3] for i in output_states[:].slip_x], "ROADVIEW4", linestyle ="--", label ="Calculated Longitudinal Slip RR")
        a3.set_xlim(xlim)
        a3.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_long_subplot-wheelLoad.png")

        f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
        a0.plot(manoeuvre.time, [i[0] for i in output_states[:].slip_y], "ROADVIEW1", linestyle ="--", label ="Calculated Lateral Slip FL")
        a0.set_xlim(xlim)
        a0.legend()
        a0.set_ylim(-0.2, 0.2)
        a0.set_xlabel("Time (s)")
        a0.set_ylabel("Wheel Slip")
        a2.set_xlabel("Time (s)")
        a2.set_ylabel("Wheel Slip")
        a2.plot(manoeuvre.time, [i[1] for i in output_states[:].slip_y], "ROADVIEW2", linestyle ="--", label ="Calculated Lateral Slip RL")
        a2.set_xlim(xlim)
        a2.legend()
        a1.set_ylim(-0.2, 0.2)
        a1.set_xlabel("Time (s)")
        a1.set_ylabel("Wheel Slip")
        a1.plot(manoeuvre.time, [i[2] for i in output_states[:].slip_y], "ROADVIEW3", linestyle ="--", label ="Calculated Lateral Slip FR")
        a1.set_xlim(xlim)
        a1.legend()
        a1.set_ylim(-0.2, 0.2)
        a3.set_ylim(-0.2, 0.2)
        a3.set_xlabel("Time (s)")
        a3.set_ylabel("Wheel Slip")
        a3.plot(manoeuvre.time, [i[3] for i in output_states[:].slip_y], "ROADVIEW4", linestyle ="--", label ="Calculated Lateral Slip RR")
        a3.set_xlim(xlim)
        a3.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_side_subplot-wheelLoad.png")
        
        plt.figure(figsize=figsize)
        plt.legend()
        plt.tight_layout()  
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_slip_side_subplot-wheelLoad.png")

        f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
        a0.plot(manoeuvre.time, [i[0] for i in output_states[:].wheel_w_vel], "ROADVIEW1", linestyle ="--", label ="Calculated Wheel Velocity FL")
        a0.set_xlim(xlim)
        a0.legend()
        a0.set_xlabel("Time (s)")
        a0.set_ylabel("Wheel Velocity")
        a2.set_xlabel("Time (s)")
        a2.set_ylabel("Wheel Velocity")
        a2.plot(manoeuvre.time, [i[1] for i in output_states[:].wheel_w_vel], "ROADVIEW2", linestyle ="--", label ="Calculated Wheel Velocity RL")
        a2.set_xlim(xlim)
        a2.legend()
        a1.set_xlabel("Time (s)")
        a1.set_ylabel("Wheel Velocity")
        a1.plot(manoeuvre.time, [i[2] for i in output_states[:].wheel_w_vel], "ROADVIEW3", linestyle ="--", label ="Calculated Wheel Velocity FR")
        a1.set_xlim(xlim)
        a1.legend()
        a3.set_xlabel("Time (s)")
        a3.set_ylabel("Wheel Velocity")
        a3.plot(manoeuvre.time, [i[3] for i in output_states[:].wheel_w_vel], "ROADVIEW4", linestyle ="--", label ="Calculated Wheel Velocity RR")
        a3.set_xlim(xlim)
        a3.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_velocity_subplot-wheelLoad.png")
        # x_rf.fx
        f, ((a0, a1),(a2,a3)) = plt.subplots(2, 2, figsize=figsize, sharex=True,sharey=True)
        
        a0.plot(manoeuvre.time, [i.fy[0] for i in output_states[:].x_rf], "ROADVIEW1", linestyle ="--", label ="Fy FL")
        a0.set_xlim(xlim)
        a0.legend()
        a0.set_xlabel("Time (s)")
        a0.set_ylabel("Wheel Forces in y")
        a2.set_xlabel("Time (s)")
        a2.set_ylabel("Wheel Forces in y")
        a2.plot(manoeuvre.time, [i.fy[1] for i in output_states[:].x_rf], "ROADVIEW2", linestyle ="--", label ="Fy RL")
        a2.set_xlim(xlim)
        a2.legend()
        a1.set_xlabel("Time (s)")
        a1.set_ylabel("Wheel Forces in y")
        a1.plot(manoeuvre.time, [i.fy[2] for i in output_states[:].x_rf], "ROADVIEW3", linestyle ="--", label ="Fy FR")
        a1.set_xlim(xlim)
        a1.legend()
        a3.set_xlabel("Time (s)")
        a3.set_ylabel("Wheel Forces in y")
        a3.plot(manoeuvre.time, [i.fy[3] for i in output_states[:].x_rf], "ROADVIEW4", linestyle ="--", label ="Fy RR")
        a3.set_xlim(xlim)
        a3.legend()
        plt.tight_layout()
        if savefig:
            plt.savefig(fig_save_dir + "/Unoptimized wheel_velocity_subplot-forcesY.png")
