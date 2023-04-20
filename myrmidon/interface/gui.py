# import logging

import matplotlib
import numpy as np
import time

matplotlib.use("TkAgg")
import threading
from functools import partial
from tkinter import *
from tkinter.ttk import *
from myrmidon.utils.misc import setup_logger
from myrmidon.utils.plotting import plot_walls, plot_assembly_area, create_goal_patch
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from myrmidon.utils import constants


class GUI:
    def __init__(self, root, group_manager, robotarium_figure, agent_positions, walls):
        self.logger = setup_logger(
            "clicked_event", constants.LOG_LOCATION + "_user-activity.log"
        )
        self.root = root
        self.root.title("Myrmidon")
        self.root.geometry("2560x1440")

        self.group_manager = group_manager
        self.robotarium_figure = robotarium_figure

        self.leader_pos_dict = {}
        self._controlled_group_ndx = 1
        self.agent_positions = agent_positions
        self.gui_override = False

        # Iniitialize for Robotarium
        print(self.robotarium_figure.set_size_inches((12, 12)))
        self.fig = Figure(figsize=(10, 10), dpi=100, facecolor="w")
        self.fig = self.robotarium_figure
        if walls is not None:
            plot_walls(walls, constants.WALL_SIZE)

        plot_assembly_area(-10, -1, 4, 2, self.fig.gca())
        thing = create_goal_patch(self.fig.gca())
        plt.close(robotarium_figure)
        self.selector = self.fig.canvas.mpl_connect(
            "button_press_event", self.mouse_click_func
        )
        # Initialize Matplotlib features
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        self.canvas.get_tk_widget().place(x=20, y=30)

        # Initialize Dropdown options
        self.formation_var = StringVar(self.root)
        self.formation_var.set("Rigid Minimal")
        x_offset = 700
        # Display Buttons, Sliders
        button_unselect = Button(
            self.root, text="Unselect", command=self.root.destroy
        ).place(x=900 + x_offset, y=30)

        button_newleader = Button(
            self.root, text="New Leader", command=self.button_newleader, state=NORMAL
        ).place(x=800 + x_offset, y=120)

        # button_join = Button(
        #     self.root, text="Join", command=self.button_join, state=NORMAL
        # ).place(x=1000, y=120)

        button_disband = Button(
            self.root, text="Disband Group", command=self.button_disband, state=NORMAL
        ).place(x=800 + x_offset, y=210)

        button_separate = Button(
            self.root, text="Separate", command=self.button_separate
        ).place(x=1000 + x_offset, y=210)

        button_add = Button(self.root, text="+", width=3, command=self.add_robot).place(
            x=960 + x_offset, y=300
        )

        entry_curr_robot_count = Entry(self.root, width=5).place(
            x=867 + x_offset, y=300
        )

        button_remove = Button(
            self.root, text="-", width=3, command=self.remove_robot
        ).place(x=925 + x_offset, y=300)

        formation_options = [
            "",
            "Rigid Minimal",
            "Complete",
            "Line",
            "Cycle",
            "Directed Cycle",
        ]
        dropdown_formation = OptionMenu(
            self.root,
            self.formation_var,
            *formation_options,
            command=self.formation_switch,
        ).place(x=900 + x_offset, y=400)

    # Define button functions

    # def button_join(self):
    #     # TODO: Implement combining
    #     print("IMPLEMENT COMBINING!!!")
    #     self.group_manager.start_barriers = not self.group_manager.start_barriers
    #     # group_manager.combine(
    #     #     main_group_id=self.controlled_group_id,
    #     #     other_group_id=selected_groups[-1]
    #     # )

    def on_click(self, x, y, button, pressed):
        if pressed:
            self.logger.info(f"clicked {button}")

    def button_separate(self):
        print("Splitting group!")
        self.logger.info("action: group split")
        self.group_manager.split(group_id=self.controlled_group_id, num_groups=2)

    def button_newleader(self):
        print("Creating New Leader!")
        self.logger.info("action: new leader")
        group_id = self.group_manager.create()
        self.group_manager.add_to_group(group_id)

    def button_disband(self):
        print("Disbanding group!")
        self.logger.info("action: group disband")
        self.group_manager.disband(group_id=self.controlled_group_id)

    def add_robot(self):
        print("Adding Robot to group!")
        self.logger.info("action: add robot")
        self.group_manager.add_to_group(group_id=self.controlled_group_id)

    def remove_robot(self):
        print("Removing Robot from group!")
        self.logger.info("action: remove robot")
        self.group_manager.remove_from_group(group_id=self.controlled_group_id)

    def formation_switch(self, graph):
        print(graph)
        self.logger.info(f"clicked change graph to {graph}")
        self.group_manager.change_group_graph(
            group_id=self.controlled_group_id, graph=graph
        )

    def next_controlled_group(self):
        self._controlled_group_ndx = (self._controlled_group_ndx + 1) % (
            len(self.group_ids)
        )
        self.logger.info(f"controlled group id: {self.controlled_group_id}")

    def mouse_click_func(self, event):

        pos = np.array([[event.xdata], [event.ydata]])
        # Left Click:
        if event.button == 1:
            if self.group_manager.leaders.size > 0:
                ndx = self.group_manager.closest_leader_to_point(
                    agent_positions=self.agent_positions, pt=pos
                )
                self._controlled_group_ndx = (
                    self._controlled_group_ndx if ndx is None else ndx
                )
                self.logger.info(f"controlled group id: {self.controlled_group_id}")

        # Right Click:
        if event.button == 3:
            print(f"Going to: {pos}")
            leader_position = self.group_leader_position(self.controlled_group_id)
            if leader_position is not None:
                self.leader_pos_dict.update({self.controlled_group_id: pos})
                self.logger.info(
                    f"action: moving group:{self.controlled_group_id} to {(','.join(map(str, pos.flatten())))}"
                )

            self.gui_override = True

        # Middle Click:
        if event.button == 2:
            if (
                self.group_manager.leaders.size > 0
                and len(self.group_manager.groups) >= 2
            ):
                ndx = self.group_manager.closest_leader_to_point(
                    agent_positions=self.agent_positions, pt=pos
                )

                if ndx is not None and ndx != self._controlled_group_ndx:
                    self.group_manager.combine(
                        main_group_id=self.controlled_group_id,
                        other_group_id=self.group_ids[ndx],
                    )

    def group_leader_position(self, group_id):
        if group_id not in self.group_manager.groups:
            return
        leader_idx = self.group_manager.groups[group_id].agents[0]
        return self.agent_positions[:, [leader_idx]]

    def update_gui(self):
        self.canvas.draw()
        self.toolbar.update()

    def get_group_id_from_ndx(self, ndx):
        if ndx >= len(self.group_ids):
            return None
        return self.group_ids[ndx]

    def update_gui_positions(self, positions):
        self.agent_positions = positions

    @property
    def leader_pos(self):
        keys = self.leader_pos_dict.keys()
        for key in list(keys):
            if key not in self.group_manager.groups:
                self.leader_pos_dict.pop(key)

        return self.leader_pos_dict

    @property
    def group_ids(self):
        return list(self.group_manager.groups.keys())

    @property
    def controlled_group_id(self):
        if not self.group_ids:
            return None
        if self._controlled_group_ndx >= len(self.group_ids):
            self.next_controlled_group()
        return self.group_ids[self._controlled_group_ndx]

    @property
    def controlled_group(self):
        if not self.group_ids:
            return None
        if self._controlled_group_ndx >= len(self.group_ids):
            self.next_controlled_group()
        return self.group_manager.groups[self.controlled_group_id]
