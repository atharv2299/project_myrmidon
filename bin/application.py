import logging
import time
from tkinter import Tk

import matplotlib.pyplot as plt
import numpy as np
import rps.robotarium as robotarium
from pynput import keyboard, mouse
from rps.utilities.barrier_certificates import *
from rps.utilities.controllers import *

from myrmidon import utils
from myrmidon.interface import GUI, TUI
from myrmidon.robots import GroupManager
from myrmidon.utils.misc import setup_logger
from myrmidon.utils.plotting import create_goal_patch, plot_assembly_area, plot_walls

plt.rcParams["keymap.save"].remove("s")
_N = 10

logger = setup_logger(
    "application", utils.constants.LOG_LOCATION + "_user-activity.log"
)
robot_position_logger = setup_logger(
    "robots", utils.constants.LOG_LOCATION + "_robot_poses.log"
)
initial_conditions = np.array(
    [
        [-9, -9, -9, -9, -9, -8, -8, -8, -8, -8],
        [0.8, 0.4, 0, -0.4, -0.8, 0.8, 0.4, 0, -0.4, -0.8],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
)

_garage = np.array(
    [
        [-9, -9, -9, -9, -9, -8, -8, -8, -8, -8],
        [0.8, 0.4, 0, -0.4, -0.8, 0.8, 0.4, 0, -0.4, -0.8],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
)

# To index: walls[wall_num][endpoint][axis]
walls = np.array(
    [
        [[-7.0, 10], [-7.0, 3]],
        [[0, 10], [0, 3]],
        [[5, 1], [8, 1]],
        [[2, -10], [2, -3]],
        [[-3.5, -10], [-3.5, -3]],
    ]
)

garage_return_controller = create_hybrid_unicycle_pose_controller(
    linear_velocity_gain=5
)
# leader_controller = create_hybrid_unicycle_pose_controller()
leader_controller = create_clf_unicycle_position_controller(linear_velocity_gain=0.6)

si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion(
    linear_velocity_gain=0.6, angular_velocity_limit=np.pi
)
_, uni_to_si_states = create_si_to_uni_mapping()
uni_to_si_dyn = create_uni_to_si_dynamics()

r = robotarium.Robotarium(
    number_of_robots=_N,
    show_figure=True,
    sim_in_real_time=False,
    initial_conditions=initial_conditions,
)
if walls is not None:
    plot_walls(walls, utils.constants.WALL_SIZE)

plot_assembly_area(r.figure.gca())
thing = create_goal_patch(r.figure.gca())
x = r.get_poses()

r.step()
root = Tk()
group_manager = GroupManager({}, _N)
tui = TUI(group_manager, True)
gui = GUI(root, group_manager, r.figure, x, walls)
leader_labels, line_follower = utils.plotting.initialize_plot(
    r, x, group_manager.num_agents
)
uni_barrier_certs = utils.custom_uni_barriers(
    safety_radius=0.12,
    projection_distance=0.05,
    group_manager=group_manager,
    connectivity_distance=2,
    barrier_gain=100,
    magnitude_limit=1,
    boundary_points=[-10, 10, -10, 10],
)

listener = keyboard.Listener(on_press=tui.on_press, suppress=False)
listener.start()
listener2 = mouse.Listener(on_click=gui.on_click, suppress=False)
listener2.start()

while not tui.exit:
    if gui.gui_override:
        if (
            tui.controlled_group_id in tui.leader_dxu
            and tui.controlled_group_id == gui.controlled_group_id
        ):
            tui.leader_dxu.pop(tui.controlled_group_id)
        gui.gui_override = False

    dxu = group_manager.get_dxu(
        tui.leader_dxu,
        _garage,
        x,
        garage_return_controller,
        leader_controller,
        si_to_uni_dyn,
        uni_to_si_dyn,
        uni_barrier_certs,
        gui.leader_pos,
        walls,
    )
    gui.update_gui()
    r.set_velocities(np.arange(_N), dxu)

    x = r.get_poses()

    gui.update_gui_positions(x)

    leader_labels, line_follower = utils.plotting.update_plot(
        group_manager,
        line_follower,
        leader_labels,
        x,
        tui.controlled_group,
        gui.controlled_group,
    )
    r.step()
root.destroy()
root.mainloop()
r.call_at_scripts_end()
