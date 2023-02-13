import matplotlib.pyplot as plt
import numpy as np
import rps.robotarium as robotarium
from pynput import keyboard
from rps.utilities.barrier_certificates import *
from rps.utilities.controllers import *

from myrmidon import utils
from myrmidon.interface import TUI
from myrmidon.robots import GroupManager

plt.rcParams["keymap.save"].remove("s")
_N = 10

group_manager = GroupManager({}, _N)
tui = TUI(group_manager)
initial_conditions = np.array(
    [
        [-1.3, -1.3, -1.3, -1.3, -1.3, 1.3, 1.3, 1.3, 1.3, 1.3],
        [0.8, 0.4, 0, -0.4, -0.8, 0.8, 0.4, 0, -0.4, -0.8],
        [0, 0, 0, 0, 0, np.pi, np.pi, np.pi, np.pi, np.pi],
    ]
)

_garage = np.array(
    [
        [-1.3, -1.3, -1.3, -1.3, -1.3, 1.3, 1.3, 1.3, 1.3, 1.3],
        [0.8, 0.4, 0, -0.4, -0.8, 0.8, 0.4, 0, -0.4, -0.8],
        [0, 0, 0, 0, 0, np.pi, np.pi, np.pi, np.pi, np.pi],
    ]
)
garage_return_controller = create_hybrid_unicycle_pose_controller()
leader_controller = create_hybrid_unicycle_pose_controller()
uni_barrier_certs = create_unicycle_barrier_certificate_with_boundary(
    safety_radius=0.12, projection_distance=0.04
)
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion(
    linear_velocity_gain=0.75, angular_velocity_limit=np.pi
)
_, uni_to_si_states = create_si_to_uni_mapping()

r = robotarium.Robotarium(
    number_of_robots=_N,
    show_figure=True,
    sim_in_real_time=False,
    initial_conditions=initial_conditions,
)
x = r.get_poses()
r.step()

leader_labels, line_follower = utils.plotting.initialize_plot(
    r, x, group_manager.num_agents
)


listener = keyboard.Listener(on_press=tui.on_press, suppress=False)
listener.start()

while True:
    dxu = group_manager.get_dxu(
        tui.leader_dxu,
        _garage,
        x,
        garage_return_controller,
        si_to_uni_dyn,
        uni_barrier_certs,
    )
    r.set_velocities(np.arange(_N), dxu)
    x = r.get_poses()

    leader_labels, line_follower = utils.plotting.update_plot(
        group_manager,
        line_follower,
        leader_labels,
        x,
        tui.controlled_group,
    )

    r.step()
