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
from myrmidon.interface import LassoGUI, TUI
from myrmidon.robots import GroupManager
from myrmidon.utils import GoalSet
from myrmidon.utils.misc import setup_logger
from myrmidon.utils.plotting import plot_assembly_area, plot_walls
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
        [[-7, 7], [-2, 7]],
        [[-5, 0], [3, 0]],
        [[3.6, -5.5], [10, -5.5]],
        [[3.6, -5.5], [3.6, -3]],
    ]
)

garage_return_controller = create_hybrid_unicycle_pose_controller(
    linear_velocity_gain=1.5, velocity_magnitude_limit=0.8
)
linvelgain = 3

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

plot_assembly_area(r.figure.gca())

completion_text = plt.text(
    0, 0, "placeholder text", size=1, ha="center", va="center", alpha=0, zorder=0
)

assembly_area_header = plt.text(
    -8,
    1.2,
    "Assembly Area",
    size=10,
    ha="center",
    va="center",
    bbox=dict(boxstyle="round", ec=(0, 0, 0), fc=(0.2, 0.8, 0.5), zorder=0, alpha=0.3),
    zorder=0,
)
x = r.get_poses()
if allow_logging:
    robot_position_logger.info(f"{','.join(map(str, x.flatten()))}")

r.step()
root = Tk()
group_manager = GroupManager({}, _N, x)
tui = TUI(group_manager, True)
gui = LassoGUI(
    root,
    group_manager,
    r.figure,
    x,
    walls,
    enable_buttons=False,
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

prev_log = time.time()
log_interval = 5

# goal_points = np.array(
#     [[5, 7], [-3.5, 6], [0, -6], [-6.5, -6], [6, -6], [8.8, 1], [-8, 8.5], [0, 0]]
# )
# num_bots_needed = np.array(
#     [
#         1,
#         2,
#         1,
#         3,
#         1,
#         5,
#         3,
#         6,
#     ]
# )
goal_point_1 = np.array(
    [
        [-8.8, 2],
        [-4.5, 5.3],
        [2.75, 2],
        [1.5, 9],
        [8.8, 0],
        [3, -5.8],
        [1.1, -8.5],
        [-0.75, -0.75],
        [-4.9, -3.1],
    ]
)
goal_pointsm = np.array(
    [[9, -6], [4, 0], [-4.4, -8.6], [-1.3, 5.3], [2.7, -2.8], [-8.5, -8.1]]
)
goal_pointmd = np.array([[-8.2, 8.6], [-0.6, 1.7], [6.5, -7.5], [8.75, 1.15]])
goal_pointlg = np.array(
    [[5, 7], [-7, -6.25], [-4.7, 6.5], [6.75, -3.1], [-0.75, -6.25]]
)

bots_per_goal_1 = np.ones(len(goal_point_1), dtype=int)
bots_per_goal_sm = np.array([1, 2, 1, 1, 1, 2])
bots_per_goal_md = np.array([3, 5, 5, 4])
bots_per_goal_lg = np.array([12, 11, 6, 10, 12])


goal_set1 = GoalSet(r.figure.gca(), goal_point_1, bots_per_goal_1, 1, x, allow_logging)
goal_set2 = GoalSet(r.figure.gca(), goal_pointsm, bots_per_goal_sm, 2, x, allow_logging)
goal_set3 = GoalSet(r.figure.gca(), goal_pointmd, bots_per_goal_md, 3, x, allow_logging)
goal_set4 = GoalSet(r.figure.gca(), goal_pointlg, bots_per_goal_lg, 4, x, allow_logging)

for i in range(_N):
    gui.button_newleader()

while not (gui.exit or tui.exit):
    goal_set1.goal_check()
    goal_set2.goal_check()
    goal_set3.goal_check()
    goal_set4.goal_check()

    if (
        goal_set1.set_complete
        and goal_set2.set_complete
        and goal_set3.set_complete
        and goal_set4.set_complete
    ):
        if allow_logging:
            logger.info(f"Experiment Completed!")
            logging.disable()
        completion_text.set(
            x=-1,
            y=0.5,
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
    goal_set1.update_robot_positions(x)
    goal_set2.update_robot_positions(x)
    goal_set3.update_robot_positions(x)
    goal_set4.update_robot_positions(x)

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
