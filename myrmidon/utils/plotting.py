from rps.utilities.misc import determine_marker_size
from myrmidon import utils
import numpy as np


def initialize_plot(r, x, num_agents):
    line_width = 3

    # INITIAL PLOTTING:
    init_L = num_agents * np.identity(num_agents) - np.ones((num_agents, num_agents))
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

    marker_size = determine_marker_size(r, utils.constants.LEADER_SELECTION_RADIUS)
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
):
    # TODO: Update plotting for GUI and TUI
    if not group_manager.groups:
        for leader_label in leader_labels:
            leader_label.set_alpha(0)
        for line in line_follower:
            line[0].set_alpha(0)
        return leader_labels, line_follower
    leaders = group_manager.leaders
    rows, cols = utils.find_connections(group_manager.block_L)
    num_bots = group_manager.num_agents

    # for i in range(len(line_follower)):
    #     line_follower[i][0].set_alpha(0)

    for line in line_follower:
        line[0].set_alpha(0)

    # TODO: Highlight controlled formation - get from UI
    # for i in range(len(leader_labels)):
    #     if i == tui_controlled_group.agents[0]:
    #     if i == tui_controlled_group.agents[0]:
    #         leader_labels[i].set_alpha(1)
    #         leader_labels[i].set_offsets(x[:2, i].T)
    #     else:
    #         leader_labels[i].set_alpha(0)

    for ndx, leader_label in enumerate(leader_labels):
        if ndx in group_manager.leaders:
            leader_label.set_alpha(1)
            leader_label.set_offsets(x[:2, ndx].T)
            # print(ndx)
            # print(tui_controlled_group.agents[0])
            # print(gui_controlled_group.agents[0])
            if tui_controlled_group:
                if ndx == tui_controlled_group.agents[0]:
                    leader_label.set_edgecolor("lime")
                else:
                    leader_label.set_edgecolor("red")
            if gui_controlled_group:
                if ndx == gui_controlled_group.agents[0]:
                    leader_label.set_facecolor("green")
                else:
                    leader_label.set_facecolor("none")
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
            return utils.constants.COLORS[key % len(utils.constants.COLORS)]
