import matplotlib
import numpy as np

matplotlib.use("TkAgg")
import threading
from functools import partial
from tkinter import *
from tkinter.ttk import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


class GUI:
    # TODO: See if keyboard is possible
    def __init__(self, root, group_manager, robotarium_figure, agent_positions):
        self.root = root
        self.group_manager = group_manager
        self.robotarium_figure = robotarium_figure
        self.root.title("Myrmidon")
        self.root.geometry("1200x600")
        self.leader_pos_dict = {}
        self._controlled_group_ndx = 0
        self.agent_positions = agent_positions
        self.gui_override = False

        # Iniitialize for Robotarium
        self.fig = Figure(figsize=(5, 5), dpi=100, facecolor="w")
        self.fig = self.robotarium_figure
        plt.close(robotarium_figure)
        self.selector = self.fig.canvas.mpl_connect(
            "button_press_event", self.mouse_click_func
        )
        # Initialize Matplotlib features
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        self.canvas.get_tk_widget().place(x=20, y=30)

        # Display Buttons, Sliders
        button_unselect = Button(
            self.root, text="Unselect", command=self.root.destroy
        ).place(x=900, y=30)

        button_newleader = Button(
            self.root, text="New Leader", command=self.button_newleader, state=NORMAL
        ).place(x=800, y=120)

        button_join = Button(
            self.root, text="Join", command=self.next_controlled_group, state=NORMAL
        ).place(x=1000, y=120)

        button_disband = Button(
            self.root, text="Disband Group", command=self.button_disband, state=NORMAL
        ).place(x=800, y=210)

        button_separate = Button(
            self.root, text="Separate", command=self.button_separate
        ).place(x=1000, y=210)

        button_add = Button(self.root, text="+", width=3, command=self.add_robot).place(
            x=800, y=300
        )

        entry_curr_robot_count = Entry(self.root, width=5).place(x=867, y=300)

        button_remove = Button(
            self.root, text="-", width=3, command=self.remove_robot
        ).place(x=925, y=300)

    # Define button functions

    def button_join(self):
        # TODO: Implement combining
        print("IMPLEMENT COMBINING!!!")
        # group_manager.combine(
        #     main_group_id=self.controlled_group_id,
        #     other_group_id=selected_groups[-1]
        # )

    def button_separate(self):
        print("Splitting group!")
        self.group_manager.split(group_id=self.controlled_group_id, num_groups=2)

    def button_newleader(self):
        print("Creating New Leader!")
        group_id = self.group_manager.create()
        self.group_manager.add_to_group(group_id)

    def button_disband(self):
        print("Disbanding group!")
        self.group_manager.disband(group_id=self.controlled_group_id)

        # self.group_manager.clear_select()

    def add_robot(self):
        print("Adding Robot to group!")
        self.group_manager.add_to_group(group_id=self.controlled_group_id)
        # self.group_manager.clear_select()

    def remove_robot(self):
        print("Removing Robot from group!")
        self.group_manager.remove_from_group(group_id=self.controlled_group_id)
        # self.group_manager.clear_select()

    def next_controlled_group(self):
        self._controlled_group_ndx = (self._controlled_group_ndx + 1) % (
            len(self.group_ids)
        )

    def mouse_click_func(self, event):
        # print(event.button)
        # print(int(event.button))
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
        # Right Click:
        if event.button == 3:
            print(f"Going to: {pos}")
            leader_position = self.group_leader_position(self.controlled_group_id)
            if leader_position is not None:
                self.leader_pos_dict.update({self.controlled_group_id: pos})
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
            print(self._controlled_group_ndx)

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
        if self._controlled_group_ndx >= len(self.group_ids):
            return None
        return self.group_ids[self._controlled_group_ndx]

    @property
    def controlled_group(self):
        if self._controlled_group_ndx >= len(self.group_ids):
            return None
        return self.group_manager.groups[self.controlled_group_id]
