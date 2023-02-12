from rps.utilities.misc import determine_marker_size
from remote_control.utils import *


def initialize_plot(L, r, x):
    N = L.shape[0]
    line_width = 3

    # INITIAL PLOTTING:
    init_L = N * np.identity(N) - np.ones((N, N))
    # completeGL(_N)
    [rows, cols] = np.where(init_L == -1)
    line_follower = [
        r.axes.plot(
            [x[0, rows[kk]], x[0, cols[kk]]],
            [x[1, rows[kk]], x[1, cols[kk]]],
            linewidth=line_width,
            color="orange",
            zorder=-1,
            alpha=0,
        )
        for kk in range(0, len((rows)))
    ]

    marker_size = determine_marker_size(r, 0.1)
    leader_labels = [
        r.axes.scatter(
            x[0, kk],
            x[1, kk],
            s=(np.pi / 4 * marker_size),
            edgecolors="r",
            facecolors="none",
            marker="o",
            zorder=0,
            alpha=0,
            linewidths=2,
        )
        for kk in range(0, 11)
    ]
    return leader_labels, line_follower


def update_plot(robot_groups, line_follower, leader_labels, x, L):
    leaders = robot_groups.leaders
    rows, cols = find_connections(L)
    num_bots = L.shape[0]

    for i in range(len(line_follower)):
        line_follower[i][0].set_alpha(0)

    for i in range(len(leader_labels)):
        if i == robot_groups.get_controlled_formation()[0]:
            leader_labels[i].set_alpha(1)
            # leader_labels[i].set_position([x[0, i], x[1, i] + 0.15])
            leader_labels[i].set_offsets(x[:2, i].T)
        else:
            leader_labels[i].set_alpha(0)

    for i, j in zip(rows, cols):

        if j in leaders:
            line_follower[i][0].set_data(
                [x[0, i], x[0, j]],
                [x[1, i], x[1, j]],
            )
            line_follower[i][0].set_alpha(1)
            line_follower[i][0].set_color(get_color(j, robot_groups.formations))
        else:
            line_follower[i * (num_bots - 1) + j][0].set_data(
                [x[0, i], x[0, j]],
                [x[1, i], x[1, j]],
            )
            line_follower[i * (num_bots - 1) + j][0].set_alpha(1)
            line_follower[i * (num_bots - 1) + j][0].set_color(
                get_color(j, robot_groups.formations)
            )
    return leader_labels, line_follower


COLORS = [
    "orange",
    "royalblue",
    "green",
    "darkslateblue",
    "purple",
    "black",
    "lime",
    "skyblue",
    "fuchsia",
    "gold",
    "white",
]


def get_color(agent_num, robot_groups):
    for key, group in robot_groups.items():
        if agent_num in group.agents:
            return COLORS[key % len(COLORS)]
