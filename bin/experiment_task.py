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
from myrmidon.utils import GoalSet
from myrmidon.utils.misc import (
    setup_logger,
    num_in_circle,
    modify_patch,
    get_circle_patch_properties,
)
from myrmidon.utils.plotting import create_goal_patch, plot_assembly_area, plot_walls
import argparse


# def goal_checking(
#     goal,
#     i,
#     x,
#     num_bots_needed,
#     goal_points,
#     goal_radius,
#     goal_checking,
#     goal_time,
#     goal_text,
#     goal_set,
#     goal_set_complete,
# ):
#     if not goal_set_complete:
#         wait_period = 5
#         center, radius = get_circle_patch_properties(goal)
#         num_goal_reached = num_in_circle(x, center, radius)
#         if num_goal_reached >= num_bots_needed[i - 1] and goal_checking:
#             goal_time = time.time()
#             modify_patch(goal, facecolor="g")
#             goal_checking = False

#         if not goal_checking:
#             if num_goal_reached < num_bots_needed[i - 1]:
#                 goal_time = time.time()
#                 modify_patch(goal, facecolor="r")
#                 goal_checking = True
#             if time.time() - goal_time >= wait_period:
#                 if i < len(goal_points):
#                     if allow_logging:
#                         logger.info(f"Finish: goal {i} of goal set {goal_set} reached")

#                     modify_patch(
#                         goal,
#                         center=goal_points[i],
#                         radius=goal_radius[i],
#                         facecolor="r",
#                     )
#                     goal_text.set(
#                         x=goal_points[i][0],
#                         y=goal_points[i][1],
#                         text=num_bots_needed[i],
#                         size=15,
#                     )
#                     i += 1
#                     if allow_logging:
#                         logger.info(f"Start: go to goal {i} of goal set {goal_set}")

#                     goal_checking = True
#                 else:
#                     if allow_logging:
#                         logger.info(f"Finish: goal {i} of goal set {goal_set} reached")

#                     modify_patch(
#                         goal,
#                         radius=0,
#                         facecolor="r",
#                     )
#                     if allow_logging:
#                         logger.info(f"Goal Set {goal_set} Completed!")
#                     goal_set_complete = True
#                     goal_text.set(
#                         x=0,
#                         y=0,
#                         text="",
#                         alpha=0,
#                         size=30,
#                     )
#     return i, goal_checking, goal_time, goal_set_complete


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
        "goal_check",
        utils.constants.LOG_LOCATION + "-" + filename + "_user-activity.log",
    )
    mouse_logger = setup_logger(
        "mouse_logger",
        utils.constants.LOG_LOCATION + "-" + filename + "_user-activity.log",
    )
    positionFormat = logging.Formatter("%(asctime)s: %(message)s")
    robot_position_logger = setup_logger(
        "robots",
        utils.constants.LOG_LOCATION + "-" + filename + "_robot_poses.log",
        formatter=positionFormat,
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
goal_points1 = goal_points[:1]
bots_per_goal1 = num_bots_needed[:1]
goal_points2 = goal_points[1:2]
bots_per_goal2 = num_bots_needed[1:2]

goal_radius1 = num_bots_needed * 0.15 + 0.5
goal_radius2 = bots_per_goal2 * 0.15 + 0.5

plot_assembly_area(r.figure.gca())
goal_patch1 = create_goal_patch(r.figure.gca(), goal_points1[0], goal_radius1[0])
goal_patch2 = create_goal_patch(r.figure.gca(), goal_points2[0], goal_radius2[0])

completion_text = plt.text(
    0, 0, "placeholder text", size=1, ha="center", va="center", alpha=0, zorder=0
)

# goal_text1 = create_text(r.figure.gca())
# goal_text1 = plt.text(
#     0, 0, "placeholder text", size=1, ha="center", va="center", zorder=0
# )
# goal_text.set(
#     x=goal_points[i][0], y=goal_points[i][1], text=num_bots_needed[i], size=15
# )

# goal_text2 = plt.text(
#     0, 0, "placeholder text", size=1, ha="center", va="center", zorder=0
# )
# goal_text2.set(
#     x=goal_points2[j][0], y=goal_points2[j][1], text=num_bots_needed2[j], size=15
# )
assembly_area_header = plt.text(
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
# if allow_logging:
#     logger.info(f"Start: go to goal {i}")
# i += 1
# j += 1
x = r.get_poses()
# robot_position_logger.info(msg="Hello!")
if allow_logging:
    robot_position_logger.info(f"{','.join(map(str, x.flatten()))}")

r.step()
root = Tk()
group_manager = GroupManager({}, _N, x)
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
    magnitude_limit=5,
    boundary_points=[-10, 10, -10, 10],
)

listener = keyboard.Listener(on_press=tui.on_press, suppress=False)
listener.start()
listener2 = mouse.Listener(on_click=gui.on_click, suppress=False)
listener2.start()
# goal_check1 = True
# goal_check2 = True
# wait_period = 5
# goal_time1 = None
# goal_time2 = None

# goal_set1_complete = False
# goal_set2_complete = False
prev_log = time.time()
log_interval = 5
goal_set1 = GoalSet(r.figure.gca(), goal_points1, bots_per_goal1, 1, x, allow_logging)
goal_set2 = GoalSet(r.figure.gca(), goal_points2, bots_per_goal2, 2, x, allow_logging)
while not tui.exit:
    goal_set1.goal_check()
    goal_set2.goal_check()
    # i, goal_check1, goal_time1, goal_set1_complete = goal_checking(
    #     goal,
    #     i,
    #     x,
    #     num_bots_needed,
    #     goal_points,
    #     goal_radius,
    #     goal_check1,
    #     goal_time1,
    #     goal_text,
    #     1,
    #     goal_set1_complete,
    # )
    # j, goal_check2, goal_time2, goal_set2_complete = goal_checking(
    #     goal2,
    #     j,
    #     x,
    #     num_bots_needed2,
    #     goal_points2,
    #     goal_radius2,
    #     goal_check2,
    #     goal_time2,
    #     goal_text2,
    #     2,
    #     goal_set2_complete,
    # )
    # center, radius = get_circle_patch_properties(goal)
    # num_goal_reached = num_in_circle(x, center, radius)
    # if num_goal_reached >= num_bots_needed[i - 1] and goal_checking:
    #     goal_time = time.time()
    #     modify_patch(goal, facecolor="g")
    #     goal_checking = False

    # if not goal_checking:
    #     if num_goal_reached < num_bots_needed[i - 1]:
    #         goal_time = time.time()
    #         modify_patch(goal, facecolor="r")
    #         goal_checking = True
    #     if time.time() - goal_time >= wait_period:
    #         if i < len(goal_points):
    #             if allow_logging:
    #                 logger.info(f"Finish: goal {i} reached")

    #             modify_patch(
    #                 goal,
    #                 center=goal_points[i],
    #                 radius=goal_radius[i],
    #                 facecolor="r",
    #             )
    #             goal_text.set(
    #                 x=goal_points[i][0],
    #                 y=goal_points[i][1],
    #                 text=num_bots_needed[i],
    #                 size=15,
    #             )
    #             i += 1
    #             if allow_logging:
    #                 logger.info(f"Start: go to goal {i}")

    #             goal_checking = True
    #         else:
    #             if allow_logging:
    #                 logger.info(f"Finish: goal {i} reached")

    #             modify_patch(
    #                 goal,
    #                 radius=0,
    #                 facecolor="r",
    #             )

    if goal_set1.set_complete and goal_set2.set_complete:
        if allow_logging:
            logger.info(f"Experiment Completed!")
        completion_text.set(
            x=0,
            y=0,
            text="Experiment Completed!",
            alpha=1,
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
    if time.time() - prev_log >= log_interval and allow_logging:
        robot_position_logger.info(f"{','.join(map(str, x.flatten()))}")
        prev_log = time.time()

    gui.update_gui_positions(x)
    group_manager.update_positions(x)

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
