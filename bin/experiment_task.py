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
from myrmidon.utils.misc import (
    setup_logger,
    num_in_circle,
    modify_patch,
    get_circle_patch_properties,
)
from myrmidon.utils.plotting import create_goal_patch, plot_assembly_area, plot_walls
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "-d",
    "--debug",
    help="Runs myrmidon in debug mode, which does nothing",
    action="store_true",
    default=False,
)

parser.add_argument(
    "-n",
    "--name",
    help="Stores the user's name to use in file nameing",
    action="append",
    default=None,
)

parser.add_argument(
    "-a",
    "--allow_logging",
    help="Allows logging",
    action="store_true",
    default=False,
)
args, _ = parser.parse_known_args()

filename = ""
if args.name is not None:
    filename = str(args.name[0])

plt.rcParams["keymap.save"].remove("s")
_N = 20
allow_logging = args.allow_logging
if allow_logging:
    logger = setup_logger(
        "application",
        utils.constants.LOG_LOCATION + "-" + filename + "_user-activity.log",
    )
    robot_position_logger = setup_logger(
        "robots", utils.constants.LOG_LOCATION + "-" + filename + "_robot_poses.log"
    )
else:
    logging.disable()

initial_conditions = np.array(
    [
        [
            -9.5,
            -9.5,
            -9.5,
            -9.5,
            -9.5,
            -9,
            -9,
            -9,
            -9,
            -9,
            -8.5,
            -8.5,
            -8.5,
            -8.5,
            -8.5,
            -8,
            -8,
            -8,
            -8,
            -8,
        ],
        [
            0.8,
            0.4,
            0,
            -0.4,
            -0.8,
            0.8,
            0.4,
            0,
            -0.4,
            -0.8,
            0.8,
            0.4,
            0,
            -0.4,
            -0.8,
            0.8,
            0.4,
            0,
            -0.4,
            -0.8,
        ],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
)

_garage = np.array(
    [
        [-9, -9, -9, -9, -9, -8, -8, -8, -8, -8],
        [0.8, 0.4, 0, -0.4, -0.8, 0.8, 0.4, 0, -0.4, -0.8],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
)
_garage = initial_conditions.copy()

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
    linear_velocity_gain=1.5, velocity_magnitude_limit=0.8
)
linvelgain = 3
# leader_controller = create_hybrid_unicycle_pose_controller()
# leader_controller = create_clf_unicycle_position_controller(linear_velocity_gain=2)
leader_controller = create_hybrid_unicycle_pose_controller(
    linear_velocity_gain=linvelgain, angular_velocity_limit=np.pi * 5
)

si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion(
    linear_velocity_gain=linvelgain, angular_velocity_limit=np.pi * 5
)
_, uni_to_si_states = create_si_to_uni_mapping()
uni_to_si_dyn = create_uni_to_si_dynamics()

r = robotarium.Robotarium(
    number_of_robots=_N,
    show_figure=True,
    sim_in_real_time=False,
    initial_conditions=initial_conditions,
)
r.axes
if walls is not None:
    plot_walls(walls, utils.constants.WALL_SIZE)
goal_points = np.array(
    [[5, 7], [-3.5, 6], [0, -6], [-6.5, -6], [6, -6], [8.8, 1], [-8, 8.5], [0, 0]]
)
goal_points = goal_points[:3]
num_bots_needed = np.array(
    [
        1,
        2,
        1,
        3,
        1,
        5,
        3,
        6,
    ]
)
num_bots_needed = num_bots_needed[:3]
goal_radius = num_bots_needed * 0.15 + 0.5
i = 0
plot_assembly_area(r.figure.gca())
goal = create_goal_patch(r.figure.gca(), goal_points[i], goal_radius[i])
goal_text = plt.text(
    0, 0, "placeholder text", size=1, ha="center", va="center", zorder=0
)
goal_text.set(
    x=goal_points[i][0], y=goal_points[i][1], text=num_bots_needed[i], size=15
)
plt.text(
    -8,
    1.2,
    "Assembly Area",
    size=10,
    ha="center",
    va="center",
    bbox=dict(
        boxstyle="round",
        ec=(0, 0, 0),
        fc=(0.2, 0.8, 0.5),
    ),
    zorder=0,
)

i += 1
x = r.get_poses()
# robot_position_logger.info(msg="Hello!")
robot_position_logger.info(f"Poses: {x}")
r.step()
root = Tk()
group_manager = GroupManager({}, _N)
tui = TUI(group_manager, True)
gui = GUI(
    root,
    group_manager,
    r.figure,
    x,
    walls,
    allow_logging=allow_logging,
    filename=filename,
)
leader_labels, line_follower = utils.plotting.initialize_plot(
    r, x, group_manager.num_agents
)
uni_barrier_certs = utils.custom_uni_barriers(
    safety_radius=0.12,
    projection_distance=0.05,
    group_manager=group_manager,
    connectivity_distance=2,
    barrier_gain=100,
    magnitude_limit=10,
    boundary_points=[-10, 10, -10, 10],
)

listener = keyboard.Listener(on_press=tui.on_press, suppress=False)
listener.start()
listener2 = mouse.Listener(on_click=gui.on_click, suppress=False)
listener2.start()
goal_checking = True
wait_period = 5
while not tui.exit:
    center, radius = get_circle_patch_properties(goal)
    num_goal_reached = num_in_circle(x, center, radius)
    if num_goal_reached >= num_bots_needed[i - 1] and goal_checking:
        goal_time = time.time()
        modify_patch(goal, facecolor="g")
        goal_checking = False

    if not goal_checking:
        if num_goal_reached < num_bots_needed[i - 1]:
            goal_time = time.time()
            modify_patch(goal, facecolor="r")
            goal_checking = True
        if time.time() - goal_time >= wait_period:
            if i < len(goal_points):
                logger.info(f"Goal {i} reached")
                modify_patch(
                    goal,
                    center=goal_points[i],
                    radius=goal_radius[i],
                    facecolor="r",
                )
                goal_text.set(
                    x=goal_points[i][0],
                    y=goal_points[i][1],
                    text=num_bots_needed[i],
                    size=15,
                )

                i += 1

                goal_checking = True
            else:
                logger.info(f"Goal {i} reached")
                modify_patch(
                    goal,
                    radius=0,
                    facecolor="r",
                )
                goal_text.set(
                    x=0,
                    y=0,
                    text="Experiment Completed!",
                    size=30,
                )

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
    robot_position_logger.info(f"Poses: {x}")

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
