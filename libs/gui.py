import tkinter as tk
from tkinter import Event, StringVar
from tkinter.constants import DISABLED
import tkinter.ttk as ttk

import time
from typing import Sequence
from scipy.spatial.transform import rotation

from setuptools.command.easy_install import main
import sim_hexa as simhexa
import math

main_window = tk.Tk()
main_window.geometry("800x550")
main_window.resizable(False, False)
main_window.title("Tests sim GUI")
main_window.columnconfigure(1, weight=1)
do_sim_tick = True
walking_speed = 1

walk_label = tk.DoubleVar()
rotate_label = tk.DoubleVar()
legs_offset_label = tk.DoubleVar()

robotMode = 'Walk'
robot_walk_mode = 'triangle'

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

def change_rotation(event=tk.Event):
    global do_walk_rotate

    if event.keysym == 'Left':
        do_walk_rotate = 1
    elif event.keysym == 'Right':
        do_walk_rotate = -1

def disable_rotation(event=tk.Event):
    global do_walk_rotate

    do_walk_rotate = 0

def change_mode(event=tk.Event):
    global robotMode
    global robot_controls_nb
    
    if event.keysym == 'r':
        robotMode = 'Rotate'
    elif event.keysym == 'w':
        robotMode = 'Walk'
        robot_controls_nb.select(0)
    elif event.keysym == 's':
        robotMode = 'Move legs'
        robot_controls_nb.select(1)
        center_robot()
    elif event.keysym == 'b':
        robotMode = 'Move body'
        robot_controls_nb.select(2)
        center_robot()
    elif event.keysym == 'e':
        robotMode = 'Easter eggs'
        robot_controls_nb.select(3)
        center_robot()

    #print(event)

def change_walk_mode():
    global robot_walk_mode

    if robot_walk_mode == 'triangle':
        robot_walk_mode = 'circle'
    elif robot_walk_mode == 'circle':
        robot_walk_mode = 'triangle'


def toggle_debug():
    global debug_state
    debug_state = not debug_state

def reset_body_position():
    global body_xpos
    global body_ypos
    global body_zpos

    body_xpos.set(0)
    body_ypos.set(0)
    body_zpos.set(0)


main_window.bind('<Return>', start_simulation)

main_window.bind('<KP_1>', change_direction)
main_window.bind('<KP_2>', change_direction)
main_window.bind('<KP_3>', change_direction)
main_window.bind('<KP_4>', change_direction)
main_window.bind('<KP_6>', change_direction)
main_window.bind('<KP_7>', change_direction)
main_window.bind('<KP_8>', change_direction)
main_window.bind('<KP_9>', change_direction)

main_window.bind('<KeyPress-Left>', change_rotation)
main_window.bind('<KeyPress-Right>', change_rotation)
main_window.bind('<KeyRelease-Left>', disable_rotation)
main_window.bind('<KeyRelease-Right>', disable_rotation)

main_window.bind_all('<r>', change_mode)
main_window.bind_all('<w>', change_mode)
main_window.bind_all('<s>', change_mode)
main_window.bind_all('<b>', change_mode)
main_window.bind_all('<e>', change_mode)

### Globals : Widgets ###
global robot_controls_nb
global start_sim_button
global walk_speed_entry
global step_lenght
global legs_offset
global reset_robot_button
global debug_state_button
global walk_mode_button

global robot_stats_group
rob_stat_mode_text = tk.StringVar()
rob_stat_pos_text = tk.StringVar()
rob_stat_orient_text = tk.StringVar()
rob_stat_frequency_text = tk.StringVar()
rob_stat_lin_speed_text = tk.StringVar()
exec_frequency = 0
global drift_label
drift_text = tk.StringVar()

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

global body_xpos
global body_ypos
global body_zpos

### Globals : Robot stats ###
global walking_direction
global robot_height
global step_height
do_walk_rotate = 0

frozen_robot_state = tk.IntVar()


egg_mode = 'None'

### Easter eggs Functions ###
def egg_wave():
    global egg_mode
    egg_mode = 'Wave'

def egg_shake():
    global egg_mode
    egg_mode = 'Shake'

def egg_twerk():
    global egg_mode
    egg_mode = 'Twerk'

def egg_madison():
    global egg_mode
    egg_mode = 'Madison' 

def gui_update_stats():
    global sim

    global robotMode
    global rob_stat_mode_text
    global rob_stat_pos_text
    global rob_stat_orient_text
    global rob_stat_lin_speed_text
    global rob_stat_frequency_text
    global exec_frequency
    global drift_text
    global drift
    global lin_speeds

    robotPos = sim.getRobotPose()

    rob_stat_mode_text.set("Current mode : {}".format(robotMode))
    rob_stat_pos_text.set("Real Position : X = {:.3f} m | Y = {:.3f} m | Z = {:.3f} m".format(robotPos[0][0], robotPos[0][1], robotPos[0][2]))
    rob_stat_orient_text.set("Orientation : Roll = {:.2f}° | Pitch = {:.2f}° | Yaw = {:.2f}°".format((robotPos[1][0]*360/(2*math.pi)), (robotPos[1][1]*360/(2*math.pi)), (robotPos[1][2]*360/(2*math.pi))))
    rob_stat_lin_speed_text.set("Linear speeds : X = {:.3f} m/s | Y = {:.3f} m/s | Z = {:.3f} m/s".format(lin_speeds[0], lin_speeds[1], lin_speeds[2]))
    rob_stat_frequency_text.set("Processing Frequency : {:.2f} Hz".format(exec_frequency))

    if drift:
        drift_text.set("Drift = YES !!")
    else:
        drift_text.set("Drift = No")

def gui_give_weight(widget, row=0, col=0):
    for i in range(0, row):
        widget.rowconfigure(i, weight=1)
    for i in range(0, col):
        widget.columnconfigure(i, weight=1)


def gui_build():
    global robot_controls_nb
    global start_sim_button
    global walk_speed_entry
    global step_lenght
    global walking_direction
    global robot_height
    global step_height
    global reset_robot_button
    global debug_state_button
    global legs_offset
    global walk_mode_button

    global robot_stats_group
    global rob_stat_mode_label
    global rob_stat_mode_text
    global rob_stat_pos_label
    global rob_stat_pos_text
    global rob_stat_orient_text
    global rob_stat_lin_speed_text
    global rob_stat_frequency_text
    global frozen_robot_state
    global drift_label
    global drift_text
    global drift

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

    global body_xpos
    global body_ypos
    global body_zpos

    robot_controls_nb = ttk.Notebook(main_window)
    tab_walk_params = ttk.Frame(robot_controls_nb)
    gui_give_weight(tab_walk_params, row=5, col=2)

    tab_leg_params = ttk.Frame(robot_controls_nb)
    gui_give_weight(tab_leg_params, row=5, col=2)

    tab_body_params = ttk.Frame(robot_controls_nb)
    gui_give_weight(tab_body_params, row=4, col=1)

    tab_easter_eggs = ttk.Frame(robot_controls_nb)
    gui_give_weight(tab_easter_eggs, row=4, col=1)

    robot_controls_nb.add(tab_walk_params, text="Walk")
    robot_controls_nb.add(tab_leg_params, text="Move leg")
    robot_controls_nb.add(tab_body_params, text="Move body")
    robot_controls_nb.add(tab_easter_eggs, text="Easter eggs")

    ### Walk Mode ###
    walk_speed_entry = tk.Scale(tab_walk_params, from_=0.01, to=3, resolution=0.01, variable=walk_label, orient='horizontal', label='Period (speed) :')
    step_lenght = tk.Scale(tab_walk_params, from_=0, to=0.25, resolution=0.01, variable=rotate_label, orient='horizontal', label='Step width :')
    step_lenght.set(0.15)
    legs_offset = tk.Scale(tab_walk_params, from_=-0.1, to=0.1, resolution=0.01, variable=legs_offset_label, orient='horizontal', label='Legs offset :')
    legs_offset.set(0.02)
    walking_direction = tk.Scale(tab_walk_params, from_=0, to=2*math.pi, resolution=(2*math.pi/360), orient='horizontal', label='Direction :')
    robot_height = tk.Scale(tab_walk_params, from_=0, to=0.1, resolution=0.01, orient='vertical')
    robot_height.set(0.05)
    step_height = tk.Scale(tab_walk_params, from_=0.0, to=0.13, resolution=0.01, orient='vertical')
    step_height.set(0.01)
    walk_mode_button = ttk.Button(tab_walk_params, text='Change walk mode', command=lambda: change_walk_mode())
    walk_mode_button.state(["disabled"])

    ### Move leg Mode ###
    legs_front_label = ttk.Label(tab_leg_params, text="Front")
    legs_back_label = ttk.Label(tab_leg_params, text="Back")
    legs_leg1_rb = tk.Radiobutton(tab_leg_params, text="Leg 1", variable=legs_legVar, value=0, highlightthickness=0)
    legs_leg2_rb = tk.Radiobutton(tab_leg_params, text="Leg 2", variable=legs_legVar, value=1, highlightthickness=0)
    legs_leg3_rb = tk.Radiobutton(tab_leg_params, text="Leg 3", variable=legs_legVar, value=2, highlightthickness=0)
    legs_leg4_rb = tk.Radiobutton(tab_leg_params, text="Leg 4", variable=legs_legVar, value=3, highlightthickness=0)
    legs_leg5_rb = tk.Radiobutton(tab_leg_params, text="Leg 5", variable=legs_legVar, value=4, highlightthickness=0)
    legs_leg6_rb = tk.Radiobutton(tab_leg_params, text="Leg 6", variable=legs_legVar, value=5, highlightthickness=0)
    legs_xpos = tk.Scale(tab_leg_params, from_=-0.2, to_=0.3, resolution=0.01, orient='horizontal', label="Target x :")
    legs_ypos = tk.Scale(tab_leg_params, from_=-0.2, to_=0.3, resolution=0.01, orient='horizontal', label="Target y :")
    legs_zpos = tk.Scale(tab_leg_params, from_=-0.2, to_=0.3, resolution=0.01, orient='horizontal', label="Target z :")

    ### Move body Mode ###
    body_xpos = tk.Scale(tab_body_params, from_=-0.2, to_=0.3, resolution=0.01, orient='horizontal', label="Target x :")
    body_ypos = tk.Scale(tab_body_params, from_=-0.2, to_=0.3, resolution=0.01, orient='horizontal', label="Target y :")
    body_zpos = tk.Scale(tab_body_params, from_=-0.2, to_=0.3, resolution=0.01, orient='horizontal', label="Target z :")
    body_reset = ttk.Button(tab_body_params, text="Reset position", command=lambda: reset_body_position())


    ### Easter Eggs Mode ###
    hello_button = ttk.Button(tab_easter_eggs, text='Wave', command=lambda: egg_wave())
    shake_button = ttk.Button(tab_easter_eggs, text='Shake', command=lambda: egg_shake())
    twerk_button = ttk.Button(tab_easter_eggs, text='Twerk', command=lambda: egg_twerk())
    madison_button = ttk.Button(tab_easter_eggs, text='Madison Dance !', command=lambda: egg_madison())

    ### Mapping stats ###
    robot_stats_group = tk.LabelFrame(main_window, text="Robot Statistics", relief='sunken', padx=5, pady=5)
    rob_stat_mode_label = ttk.Label(robot_stats_group, textvariable=rob_stat_mode_text)
    rob_stat_pos_label = ttk.Label(robot_stats_group, textvariable=rob_stat_pos_text)
    rob_stat_orient_label = ttk.Label(robot_stats_group, textvariable=rob_stat_orient_text)
    rob_stat_lin_speed_label = ttk.Label(robot_stats_group, textvariable=rob_stat_lin_speed_text)
    rob_stat_frequency_label = ttk.Label(robot_stats_group, textvariable=rob_stat_frequency_text)
    drift_label = ttk.Label(robot_stats_group, textvariable=drift_text)

    ### Global widgets ###
    start_sim_button = ttk.Button(main_window, text="Pause Simulation", command=lambda: start_simulation())
    reset_robot_button = ttk.Button(main_window, text="Reset Robot", command=lambda: center_robot())
    frozen_robot_checkbox = ttk.Checkbutton(main_window, text="Freeze Robot", variable=frozen_robot_state)
    debug_state_button = ttk.Button(main_window, text="Toggle Debug", command=lambda: toggle_debug())

    ### Building GUI controls ###
    robot_controls_nb.grid(row=0, column=0, padx=5, pady=5)

    ### Walk Widgets Gridding ###
    walk_speed_entry.grid(row=0, column=0, padx=20, pady=10)
    step_lenght.grid(row=1, column=0, padx=20, pady=10)
    walking_direction.grid(row=2, column=0, padx=20, pady=10)
    robot_height.grid(row=0, column=1, rowspan=2)
    step_height.grid(row=2, column=1, rowspan=2)
    legs_offset.grid(row=3, column=0)
    walk_mode_button.grid(row=4, column=0, columnspan=2, sticky='ew', padx=5, pady=5)

    ### ComputeIK Widgets Gridding ###
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

    ### Move body static gridding ###
    body_xpos.grid(row=0, column=0, sticky='ew')
    body_ypos.grid(row=1, column=0, sticky='ew')
    body_zpos.grid(row=2, column=0, sticky='ew')
    body_reset.grid(row=3, column=0, sticky='ew')

    ### Easter Eggs Widgets Gridding ###
    hello_button.grid(row=0, column=0, sticky='ew')
    shake_button.grid(row=1, column=0, sticky='ew')
    twerk_button.grid(row=2, column=0, sticky='ew')
    madison_button.grid(row=3, column=0, sticky='ew')
    
    ### Global widgets ###
    start_sim_button.grid(row=1, column=0, padx=5, pady=5, sticky='ew')
    reset_robot_button.grid(row=1, column=1, padx=5, pady=5, sticky='ew')
    frozen_robot_checkbox.grid(row=2, column=0, padx=5, pady=5)
    debug_state_button.grid(row=2, column=1, padx=5, pady=5, sticky='ew')

    ### Bulding Stats Book ###
    rob_stat_mode_label.grid(row=0, column=0, sticky='nw')
    rob_stat_pos_label.grid(row=1, column=0, sticky='nw')
    rob_stat_orient_label.grid(row=2, column=0, sticky='nw')
    rob_stat_lin_speed_label.grid(row=3, column=0, sticky='nw')
    rob_stat_frequency_label.grid(row=4, column=0, sticky='nw')
    drift_label.grid(row=5, column=0, sticky='nw')

    robot_stats_group.grid(row=0, column=1, padx=5, pady=5, sticky='nesw')


gui_build()

controls, sim, pos, leg_center_pos, leg_angle, targets = simhexa.init_simulation()
debug_state = False

walk_speed_entry.set(1.5)

sim_start_time = time.time()
old_refresh_time = 0

send_drift=False

lin_speeds = [0, 0, 0]

drift = False

while True:

    entry_time = time.time() - sim_start_time

    main_window.update()

    frozen = frozen_robot_state.get()
    
    if do_sim_tick == True:
        if robotMode == 'Walk':
            drift, lin_speeds = simhexa.walk(sim, targets, 
                                            speed=float(walk_speed_entry.get()),
                                            step_lenght=float(step_lenght.get()),
                                            direction=float(walking_direction.get()), 
                                            robot_height=float(robot_height.get()), 
                                            step_height=float(step_height.get()),
                                            debug=debug_state,
                                            frozen=frozen,
                                            send_drift=send_drift,
                                            legs_offset=legs_offset.get(),
                                            walk_mode=robot_walk_mode,
                                            rotate=do_walk_rotate)


        elif robotMode == 'Rotate':
            simhexa.rotate(sim, targets, 
                            speed=float(walk_speed_entry.get()), 
                            direction=float(walking_direction.get()), 
                            robot_height=float(robot_height.get()), 
                            step_height=float(step_height.get()), 
                            debug=debug_state,
                            frozen=frozen)

        elif robotMode == 'Move legs':
            simhexa.move_legs(sim, targets,
                           legID=legs_legVar.get(), 
                           leg_target_x=legs_xpos.get(), 
                           leg_target_y=legs_ypos.get(), 
                           leg_target_z=legs_zpos.get(), 
                           debug=debug_state,
                           frozen=frozen)

        elif robotMode == 'Move body':
            simhexa.move_body(sim, targets,
                           body_target_x=body_xpos.get(), 
                           body_target_y=body_ypos.get(), 
                           body_target_z=body_zpos.get(), 
                           debug=debug_state,
                           frozen=frozen)

        elif robotMode == 'Easter eggs':
            if egg_mode == 'Wave':
                simhexa.egg_wave(sim, targets, frozen=frozen)

            elif egg_mode == 'Shake':
                simhexa.egg_shake(sim, targets, frozen=frozen)

            elif egg_mode == 'Twerk':
                simhexa.egg_twerk(sim, targets, frozen=frozen)

            elif egg_mode == 'Madison':
                simhexa.egg_madison(sim, targets)

    exit_time = time.time() - sim_start_time

    if (time.time() - old_refresh_time >= 0.1):
        exec_frequency = 1/(exit_time-entry_time)
        gui_update_stats()

        send_drift = not send_drift
        
        old_refresh_time = time.time()
