import tkinter as tk
from tkinter import Event, StringVar
import tkinter.ttk as ttk

import time

from setuptools.command.easy_install import main
import sim_hexa as simhexa
import math

main_window = tk.Tk()
main_window.geometry("650x450")
main_window.resizable(False, False)
main_window.title("Tests sim GUI")
main_window.columnconfigure(1, weight=1)
do_sim_tick = True
walking_speed = 1

walk_label = tk.DoubleVar()

robotMode = 'Walk'

def start_simulation(event=tk.Event):
    global do_sim_tick
    do_sim_tick = not do_sim_tick

def center_robot():
    global sim, targets
    simhexa.reset_robot(sim, targets)

def change_direction(event=tk.Event):
    global walking_direction

    if event.keysym == 'KP_1':
        walking_direction.set(3*math.pi/4)
    elif event.keysym == 'KP_2':
        walking_direction.set(math.pi)
    elif event.keysym == 'KP_3':
        walking_direction.set(5*math.pi/4)
    elif event.keysym == 'KP_4':
        walking_direction.set(math.pi/2)
    elif event.keysym == 'KP_6':
        walking_direction.set(3*math.pi/2)
    elif event.keysym == 'KP_7':
        walking_direction.set(math.pi/4)
    elif event.keysym == 'KP_8':
        walking_direction.set(0)
    elif event.keysym == 'KP_9':
        walking_direction.set(7*math.pi/4)

def change_mode(event=tk.Event):
    global robotMode
    
    if event.keysym == 'r':
        robotMode = 'Rotate'
    elif event.keysym == 'w':
        robotMode = 'Walk'
    elif event.keysym == 's':
        robotMode = 'Static'
        center_robot()

    print(event)

def toggle_debug():
    global debug_state
    debug_state = not debug_state


main_window.bind('<Return>', start_simulation)

main_window.bind('<KP_1>', change_direction)
main_window.bind('<KP_2>', change_direction)
main_window.bind('<KP_3>', change_direction)
main_window.bind('<KP_4>', change_direction)
main_window.bind('<KP_6>', change_direction)
main_window.bind('<KP_7>', change_direction)
main_window.bind('<KP_8>', change_direction)
main_window.bind('<KP_9>', change_direction)

main_window.bind_all('<r>', change_mode)
main_window.bind_all('<w>', change_mode)
main_window.bind_all('<s>', change_mode)

### Globals : Widgets ###
global robot_controls_nb
global start_sim_button
global walk_speed_entry
global reset_robot_button
global debug_state_button

global robot_stats_group
rob_stat_mode_text = tk.StringVar()
rob_stat_pos_text = tk.StringVar()
rob_stat_orient_text = tk.StringVar()
rob_stat_frequency_text = tk.StringVar()
exec_frequency = 0

global legs_leg1_rb
global legs_leg2_rb
global legs_leg3_rb
global legs_leg4_rb
global legs_leg5_rb
global legs_leg6_rb
legs_legVar = tk.IntVar()
legs_legVar.set(0)
global legs_xpos
global legs_ypos
global legs_zpos

### Globals : Robot stats ###
global walking_direction
global robot_height
global step_height

frozen_robot_state = tk.IntVar()

def gui_update_stats():
    global sim

    global robotMode
    global rob_stat_mode_text
    global rob_stat_pos_text
    global rob_stat_orient_text
    global rob_stat_frequency_text
    global exec_frequency

    robotPos = sim.getRobotPose()

    rob_stat_mode_text.set("Current mode : {}".format(robotMode))
    rob_stat_pos_text.set("Real Position : X = {:.3f} m | Y = {:.3f} m | Z = {:.3f} m".format(robotPos[0][0], robotPos[0][1], robotPos[0][2]))
    rob_stat_orient_text.set("Orientation : Roll = {:.2f}° | Pitch = {:.2f}° | Yaw = {:.2f}°".format((robotPos[1][0]*360/(2*math.pi)), (robotPos[1][1]*360/(2*math.pi)), (robotPos[1][2]*360/(2*math.pi))))
    rob_stat_frequency_text.set("Processing Frequency : {:.2f} Hz".format(exec_frequency))

def gui_give_weight(widget, row=0, col=0):
    for i in range(0, row):
        widget.rowconfigure(i, weight=1)
    for i in range(0, col):
        widget.columnconfigure(i, weight=1)


def gui_build():
    global robot_controls_nb
    global start_sim_button
    global walk_speed_entry
    global walking_direction
    global robot_height
    global step_height
    global reset_robot_button
    global debug_state_button

    global robot_stats_group
    global rob_stat_mode_label
    global rob_stat_mode_text
    global rob_stat_pos_label
    global rob_stat_pos_text
    global rob_stat_orient_text
    global rob_stat_frequency_text
    global frozen_robot_state

    global legs_leg1_rb
    global legs_leg2_rb
    global legs_leg3_rb
    global legs_leg4_rb
    global legs_leg5_rb
    global legs_leg6_rb
    global legs_legVar
    global legs_xpos
    global legs_ypos
    global legs_zpos

    robot_controls_nb = ttk.Notebook(main_window)
    tab_walk_params = ttk.Frame(robot_controls_nb)

    tab_leg_params = ttk.Frame(robot_controls_nb)
    gui_give_weight(tab_leg_params, row=5, col=2)

    robot_controls_nb.add(tab_walk_params, text="Walk")
    robot_controls_nb.add(tab_leg_params, text="Move leg")

    ### Walk Mode ###
    start_sim_button = ttk.Button(tab_walk_params, text="Start Simulation", command=lambda: start_simulation())
    walk_speed_entry = tk.Scale(tab_walk_params, from_=0.01, to=3, resolution=0.01, variable=walk_label, orient='horizontal')
    walking_direction = tk.Scale(tab_walk_params, from_=0, to=2*math.pi, resolution=(2*math.pi/360), orient='horizontal')
    robot_height = tk.Scale(tab_walk_params, from_=0, to=0.1, resolution=0.01, orient='vertical')
    robot_height.set(0.05)
    step_height = tk.Scale(tab_walk_params, from_=0.0, to=0.13, resolution=0.01, orient='vertical')
    step_height.set(0.01)

    ### Move leg Mode ###
    legs_front_label = ttk.Label(tab_leg_params, text="Front")
    legs_back_label = ttk.Label(tab_leg_params, text="Back")
    legs_leg1_rb = tk.Radiobutton(tab_leg_params, text="Leg 1", variable=legs_legVar, value=0, highlightthickness=0)
    legs_leg2_rb = tk.Radiobutton(tab_leg_params, text="Leg 2", variable=legs_legVar, value=1, highlightthickness=0)
    legs_leg3_rb = tk.Radiobutton(tab_leg_params, text="Leg 3", variable=legs_legVar, value=2, highlightthickness=0)
    legs_leg4_rb = tk.Radiobutton(tab_leg_params, text="Leg 4", variable=legs_legVar, value=3, highlightthickness=0)
    legs_leg5_rb = tk.Radiobutton(tab_leg_params, text="Leg 5", variable=legs_legVar, value=4, highlightthickness=0)
    legs_leg6_rb = tk.Radiobutton(tab_leg_params, text="Leg 6", variable=legs_legVar, value=5, highlightthickness=0)
    legs_xpos = tk.Scale(tab_leg_params, from_=-0.2, to_=0.2, resolution=0.01, orient='horizontal', label="Target x :")
    legs_ypos = tk.Scale(tab_leg_params, from_=-0.2, to_=0.2, resolution=0.01, orient='horizontal', label="Target y :")
    legs_zpos = tk.Scale(tab_leg_params, from_=-0.2, to_=0.2, resolution=0.01, orient='horizontal', label="Target z :")

    ### Mapping stats ###
    robot_stats_group = tk.LabelFrame(main_window, text="Robot Statistics", relief='sunken', padx=5, pady=5)
    rob_stat_mode_label = ttk.Label(robot_stats_group, textvariable=rob_stat_mode_text)
    rob_stat_pos_label = ttk.Label(robot_stats_group, textvariable=rob_stat_pos_text)
    rob_stat_orient_label = ttk.Label(robot_stats_group, textvariable=rob_stat_orient_text)
    rob_stat_frequency_label = ttk.Label(robot_stats_group, textvariable=rob_stat_frequency_text)

    ### Global widgets ###
    reset_robot_button = ttk.Button(main_window, text="Reset Robot", command=lambda: center_robot())
    frozen_robot_checkbox = ttk.Checkbutton(main_window, text="Freeze Robot", variable=frozen_robot_state)
    debug_state_button = ttk.Button(main_window, text="Toggle Debug", command=lambda: toggle_debug())

    ### Building GUI controls ###
    robot_controls_nb.grid(row=0, column=0, padx=5, pady=5)

    start_sim_button.grid(padx=20, pady=20)
    walk_speed_entry.grid(row=1, column=0, padx=20, pady=10)
    walking_direction.grid(row=2, column=0, padx=20, pady=10)
    robot_height.grid(row=1, column=1)
    step_height.grid(row=2, column=1)


    legs_front_label.grid(row=0, column=0, columnspan=2)
    legs_radio_baserow = 1
    legs_leg1_rb.grid(row=legs_radio_baserow, column=1, padx=5, pady=5)
    legs_leg2_rb.grid(row=legs_radio_baserow, column=0, padx=5, pady=5)
    legs_leg3_rb.grid(row=legs_radio_baserow+1, column=0, padx=5, pady=5)
    legs_leg4_rb.grid(row=legs_radio_baserow+2, column=0, padx=5, pady=5)
    legs_leg5_rb.grid(row=legs_radio_baserow+2, column=1, padx=5, pady=5)
    legs_leg6_rb.grid(row=legs_radio_baserow+1, column=1, padx=5, pady=5)
    legs_back_label.grid(row=legs_radio_baserow+3, column=0, columnspan=2)
    legs_cursors_baserow = legs_radio_baserow+4
    legs_xpos.grid(row=legs_cursors_baserow, column=0, columnspan=2, sticky='ew')
    legs_ypos.grid(row=legs_cursors_baserow+1, column=0, columnspan=2, sticky='ew')
    legs_zpos.grid(row=legs_cursors_baserow+2, column=0, columnspan=2, sticky='ew')

    
    ### Global widgets ###
    reset_robot_button.grid(row=1, column=0, padx=5, pady=5, columnspan=2, sticky='ew')
    frozen_robot_checkbox.grid(row=2, column=0)
    debug_state_button.grid(row=2, column=1, padx=5, pady=5)

    ### Bulding Stats Book ###
    rob_stat_mode_label.grid(row=0, column=0, sticky='nw')
    rob_stat_pos_label.grid(row=1, column=0, sticky='nw')
    rob_stat_orient_label.grid(row=2, column=0, sticky='nw')
    rob_stat_frequency_label.grid(row=3, column=0, sticky='nw')
    robot_stats_group.grid(row=0, column=1, padx=5, pady=5, sticky='nesw')


gui_build()

controls, sim, pos, leg_center_pos, leg_angle, targets = simhexa.init_simulation()
debug_state = False

walk_speed_entry.set(1.5)

sim_start_time = time.time()
old_refresh_time = 0

while True:
    entry_time = time.time() - sim_start_time

    main_window.update()

    frozen = frozen_robot_state.get()
    
    if do_sim_tick == True:
        if robotMode == 'Walk':
            simhexa.walk(sim, targets, 
                        speed=float(walk_speed_entry.get()), 
                        direction=float(walking_direction.get()), 
                        robot_height=float(robot_height.get()), 
                        step_height=float(step_height.get()), 
                        debug=debug_state,
                        frozen=frozen)


        elif robotMode == 'Rotate':
            simhexa.rotate(sim, targets, debug=debug_state, frozen=frozen)

        elif robotMode == 'Static':
            simhexa.static(sim, targets,
                           legID=legs_legVar.get(), 
                           leg_target_x=legs_xpos.get(), 
                           leg_target_y=legs_ypos.get(), 
                           leg_target_z=legs_zpos.get(), 
                           debug=debug_state,
                           frozen=frozen)

    exit_time = time.time() - sim_start_time

    if (time.time() - old_refresh_time >= 0.1):
        exec_frequency = 1/(exit_time-entry_time)
        gui_update_stats()
        
        old_refresh_time = time.time()
