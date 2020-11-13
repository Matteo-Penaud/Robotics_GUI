from os import walk
import tkinter as tk
from tkinter import Event
import tkinter.ttk as ttk
import sim_hexa as simhexa
import math

main_window = tk.Tk()
main_window.title("Tests sim GUI")
do_sim_tick = False
walking_speed = 1

walk_label = tk.DoubleVar()

def start_simulation(event=tk.Event):
    global do_sim_tick
    do_sim_tick = not do_sim_tick

def center_robot():
    global sim
    simhexa.reset_robot(sim)

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


main_window.bind('<Return>', start_simulation)

main_window.bind('<KP_1>', change_direction)
main_window.bind('<KP_2>', change_direction)
main_window.bind('<KP_3>', change_direction)
main_window.bind('<KP_4>', change_direction)
main_window.bind('<KP_6>', change_direction)
main_window.bind('<KP_7>', change_direction)
main_window.bind('<KP_8>', change_direction)
main_window.bind('<KP_9>', change_direction)

start_sim_button = ttk.Button(main_window, text="Start Simulation", command=lambda: start_simulation())
walk_speed_entry = tk.Scale(main_window, from_=0.5, to=3, resolution=0.1, variable=walk_label, orient='horizontal')
walking_direction = tk.Scale(main_window, from_=0, to=2*math.pi, resolution=(2*math.pi/360), orient='horizontal')
robot_height = tk.Scale(main_window, from_=0, to=0.1, resolution=0.01, orient='vertical')
step_height = tk.Scale(main_window, from_=-0.130, to=0, resolution=0.01, orient='vertical')
reset_robot_button = ttk.Button(main_window, text="Reset Robot", command=lambda: center_robot())

start_sim_button.grid(padx=20, pady=20)
walk_speed_entry.grid(row=1, column=0, padx=20, pady=10)
walking_direction.grid(row=2, column=0, padx=20, pady=10)
robot_height.grid(row=1, column=1)
step_height.grid(row=2, column=1)
reset_robot_button.grid(row=3, column=0)

controls, sim, pos, leg_center_pos, leg_angle, targets = simhexa.init_simulation()

walk_speed_entry.set(1.5)

while True:
    main_window.update()

    if do_sim_tick == True:
        simhexa.walk(sim, targets, speed=float(walk_speed_entry.get()), direction=float(walking_direction.get()), robot_height=float(robot_height.get()), step_height=float(step_height.get()))
