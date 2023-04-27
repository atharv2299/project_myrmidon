from rps.utilities.misc import determine_marker_size
from myrmidon import utils
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def initialize_plot(r, x, num_agents):
    line_width = 3

    # INITIAL PLOTTING:
    init_L = num_agents * np.identity(num_agents) - np.ones((num_agents, num_agents))
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

    marker_size = determine_marker_size(r, utils.constants.LEADER_SELECTION_RADIUS)
    leader_labels = [
        r.axes.scatter(
            x[0, kk],
            x[1, kk],
            s=(np.pi / 4 * marker_size),
            edgecolors="r",
            facecolors="none",
            marker="o",
            zorder=0,
            alpha=1,
            linewidths=2,
        )
        for kk in range(0, num_agents)
    ]
    return leader_labels, line_follower


def update_plot(
    group_manager,
    line_follower,
    leader_labels,
    x,
    tui_controlled_group,
    gui_controlled_group,
    gui_selected_agents=None,
):
    if not group_manager.groups:
        for leader_label in leader_labels:
            leader_label.set_alpha(0)
        for line in line_follower:
            line[0].set_alpha(0)
        return leader_labels, line_follower
    leaders = group_manager.leaders
    rows, cols = utils.find_connections(group_manager.block_L)
    num_bots = group_manager.num_agents

    for line in line_follower:
        line[0].set_alpha(0)

    for ndx, leader_label in enumerate(leader_labels):
        if ndx in group_manager.leaders:
            leader_label.set_alpha(1)
            leader_label.set_offsets(x[:2, ndx].T)

            # if tui_controlled_group:
            #     if ndx == tui_controlled_group.agents[0]:
            #         leader_label.set_edgecolor("lime")
            #     else:
            #         leader_label.set_edgecolor("red")
            if gui_controlled_group:
                if ndx == gui_controlled_group.agents[0]:
                    leader_label.set_edgecolor("lime")
                    leader_label.set_facecolor("blue")

                    if gui_selected_agents is not None and len(gui_selected_agents) > 0:
                        leader_label.set_edgecolor("red")
                else:
                    leader_label.set_edgecolor("red")
                    leader_label.set_facecolor("none")
            if gui_selected_agents is not None and ndx in gui_selected_agents:
                # if ndx == gui_controlled_group.agents[0]:
                leader_label.set_edgecolor("lime")
                # else:
                #     leader_label.set_edgecolor("none")
            # else:
            #     leader_label.set_edgecolor("red")

        else:
            leader_label.set_alpha(0)

    for i, j in zip(rows, cols):

        if j in leaders:
            line_follower[i][0].set_data(
                [x[0, i], x[0, j]],
                [x[1, i], x[1, j]],
            )
            line_follower[i][0].set_alpha(1)
            line_follower[i][0].set_color(get_color(j, group_manager))
        else:
            line_follower[i * (num_bots - 1) + j][0].set_data(
                [x[0, i], x[0, j]],
                [x[1, i], x[1, j]],
            )
            line_follower[i * (num_bots - 1) + j][0].set_alpha(1)
            line_follower[i * (num_bots - 1) + j][0].set_color(
                get_color(j, group_manager)
            )
    return leader_labels, line_follower


def get_color(agent_num, group_manager):
    for key, group in group_manager.groups.items():
        if agent_num in group.agents:
            return utils.constants.COLORS[key % len(utils.constants.COLORS)]
    print(f"{agent_num} is not in a group")


def plot_assembly_area(ax):
    x, y, w, h = utils.constants.ASSEMBLY_AREA
    assembly_area = plt.Rectangle(
        (x, y), w, h, edgecolor="k", facecolor="g", alpha=0.2, zorder=0
    )
    ax.add_patch(assembly_area)


def create_goal_patch(ax, center, radius):
    goal_patch = plt.Circle(
        center, radius, alpha=0.5, edgecolor="k", facecolor="r", zorder=0
    )
    ax.add_patch(goal_patch)
    return goal_patch


def create_text(ax):
    text = ax.text(
        0, 0, "placeholder text", size=1, ha="center", va="center", alpha=0, zorder=0
    )
    return text


def modify_patch(
    patch,
    center=None,
    edgecolor=None,
    facecolor=None,
    radius=None,
    alpha=None,
):
    if center is not None:
        patch.set(center=center)
    if edgecolor is not None:
        patch.set(edgecolor=edgecolor)
    if facecolor is not None:
        patch.set(facecolor=facecolor)
    if radius is not None:
        patch.set(radius=radius)
    if alpha is not None:
        patch.set(alpha=alpha)


def plot_walls(walls, wall_size, color="k"):
    for wall in walls:
        if wall.shape == (2, 2):
            x = [wall[0][0], wall[1][0]]
            y = [wall[0][1], wall[1][1]]
            plt.plot(x, y, color, linewidth=wall_size)


def adjust_patch(patch):
    pass
