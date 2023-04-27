import logging

import matplotlib
import numpy as np
import time

matplotlib.use("TkAgg")
import threading
from functools import partial
from tkinter import *
from tkinter.ttk import *
from myrmidon.utils.misc import setup_logger, in_box_area
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from myrmidon.utils import constants


def allow_operation(func):
    def wrapper(self, *args, **kwargs):
        leader_pose = self.group_leader_position(self.controlled_group_id)
        allow = in_box_area(constants.ASSEMBLY_AREA, leader_pose)
        # if func.__name__
        if allow:
            func(self, *args, **kwargs)
        return

    return wrapper


class GUI:
    def __init__(
        self,
        root,
        group_manager,
        robotarium_figure,
        agent_positions,
        walls,
        enable_buttons=True,
        allow_logging=True,
        filename="",
    ):
        self.allow_logging = allow_logging
        if self.allow_logging:
            # self.logger = setup_logger(
            #     "mouse_logger",
            #     constants.LOG_LOCATION + "-" + filename + "_user-activity.log",
            # )
            self.logger = logging.getLogger("mouse_logger")
        self.root = root
        self.root.title("Myrmidon")
        self.root.geometry("2560x1440")
        self.exit = False

        self.group_manager = group_manager
        self.robotarium_figure = robotarium_figure

        self.leader_pos_dict = {}
        self._controlled_group_ndx = 1
        self.agent_positions = agent_positions
        self.gui_override = False

        # Iniitialize for Robotarium
        self.robotarium_figure.set_size_inches((12, 12))
        self.fig = Figure(figsize=(10, 10), dpi=100, facecolor="w")
        self.fig = self.robotarium_figure
        plt.close(robotarium_figure)
        # self.selector = self.fig.canvas.mpl_connect(
        #     "button_press_event", self.mouse_click_func
        # )
        self.rect = plt.Rectangle((0, 0), 1, 1, alpha=0)
        self.x0 = None
        self.y0 = None
        self.x1 = None
        self.y1 = None
        self.fig.gca().add_patch(self.rect)
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.update_rect = False
        self.selected_agents = np.array([])
        # Initialize Matplotlib features
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        self.canvas.get_tk_widget().place(x=20, y=30)

        # Initialize Dropdown options
        self.formation_var = StringVar(self.root)
        self.formation_var.set("Rigid Minimal")
        x_offset = 700
        button_unselect = Button(self.root, text="Exit", command=self.end_gui).place(
            x=900 + x_offset, y=30
        )
        if enable_buttons:
            # Display Buttons, Sliders

            button_newleader = Button(
                self.root,
                text="New Leader",
                command=self.button_newleader,
                state=NORMAL,
            ).place(x=800 + x_offset, y=120)

            # button_join = Button(
            #     self.root, text="Join", command=self.button_join, state=NORMAL
            # ).place(x=1000, y=120)

            button_disband = Button(
                self.root,
                text="Disband Group",
                command=self.button_disband,
                state=NORMAL,
            ).place(x=800 + x_offset, y=210)

            button_separate = Button(
                self.root, text="Separate", command=self.button_separate
            ).place(x=1000 + x_offset, y=210)

            button_add = Button(
                self.root, text="+", width=3, command=self.add_robot
            ).place(x=960 + x_offset, y=300)

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
    def end_gui(self):
        self.exit = True

    def on_press(self, event):
        # print("press")
        pos = np.array([[event.xdata], [event.ydata]])
        if event.button == 1:
            self.selected_agents = np.array([])
            self.x0 = event.xdata
            self.y0 = event.ydata
            self.update_rect = True
            self.rect.set(alpha=0)
            self.rect.set_xy((self.x0, self.y0))
            self.fig.canvas.draw()
            self.select_leader(pos)
        if event.button == 3:
            if len(self.selected_agents) == 0:
                self.drive_individual_to_point(pos)
            else:
                self.drive_to_point(pos)

    def on_release(self, event):
        if event.button == 1:
            # print("release")
            self.x1 = event.xdata
            self.y1 = event.ydata
            self.in_area()
            self.rect.set_width(self.x1 - self.x0)
            self.rect.set_height(self.y1 - self.y0)
            self.rect.set(alpha=0)
            self.fig.canvas.draw()
            self.update_rect = False

    def on_motion(self, event):
        if self.update_rect:
            x1 = event.xdata
            y1 = event.ydata
            self.rect.set(width=(x1 - self.x0), height=(y1 - self.y0))
            # self.rect.set_height(y1 - self.y0)
            # self.rect.set_xy((self.x0, self.y0))
            self.rect.set(alpha=0.2)
            self.fig.canvas.draw()

    def in_area(self):
        x_min = min(self.x0, self.x1)
        x_max = max(self.x0, self.x1)
        y_min = min(self.y0, self.y1)
        y_max = max(self.y0, self.y1)
        for ndx, agent in enumerate(self.agent_positions.T):
            if x_min <= agent[0] <= x_max and y_min <= agent[1] <= y_max:
                self.selected_agents = np.append(self.selected_agents, ndx)
                # self.selected_agent_ids = np.append(self.selected_agent_ids, agent)
        # print(self.selected_agents)

    def on_click(self, x, y, button, pressed):
        if pressed:
            if self.allow_logging:
                self.logger.info(f"clicked {button}")

    @allow_operation
    def button_separate(self):
        print("Splitting group!")
        if self.allow_logging:
            self.logger.info("action: group split")
        self.group_manager.split(group_id=self.controlled_group_id, num_groups=2)

    def button_newleader(self):
        print("Creating New Leader!")
        if self.allow_logging:
            self.logger.info("action: new leader")
        group_id = self.group_manager.create()
        self.group_manager.add_to_group(group_id)

    def button_disband(self):
        print("Disbanding group!")
        if self.allow_logging:
            self.logger.info("action: group disband")
        self.group_manager.disband(group_id=self.controlled_group_id)

    @allow_operation
    def add_robot(self):
        print("Adding Robot to group!")
        if self.allow_logging:
            self.logger.info("action: add robot")
        self.group_manager.add_to_group(group_id=self.controlled_group_id)

    def remove_robot(self):
        print("Removing Robot from group!")
        if self.allow_logging:
            self.logger.info("action: remove robot")
        self.group_manager.remove_from_group(group_id=self.controlled_group_id)

    def formation_switch(self, graph):
        if self.allow_logging:
            self.logger.info(f"clicked change graph to {graph}")
        self.group_manager.change_group_graph(
            group_id=self.controlled_group_id, graph=graph
        )

    def next_controlled_group(self):
        self._controlled_group_ndx = (self._controlled_group_ndx + 1) % (
            len(self.group_ids)
        )
        if self.allow_logging:
            self.logger.info(f"controlled group id: {self.controlled_group_id}")

    def mouse_click_func(self, event):

        pos = np.array([[event.xdata], [event.ydata]])
        # Left Click:
        if event.button == 1:
            self.select_leader(pos)

        # Right Click:
        if event.button == 3:
            self.drive_to_point(pos)
        # Middle Click:
        if event.button == 2:
            self.join_groups(pos)

    def select_leader(self, pos):
        if self.group_manager.leaders.size > 0:
            ndx = self.group_manager.closest_leader_to_point(
                agent_positions=self.agent_positions, pt=pos
            )
            self._controlled_group_ndx = (
                self._controlled_group_ndx if ndx is None else ndx
            )
            if self.allow_logging:
                self.logger.info(f"controlled group id: {self.controlled_group_id}")

    def drive_individual_to_point(self, pos):
        leader_position = self.group_leader_position(self.controlled_group_id)
        if leader_position is not None:
            self.leader_pos_dict.update(
                {self.controlled_group_id: np.append(pos, 0).reshape((-1, 1))}
            )
            if self.allow_logging:
                self.logger.info(
                    f"action: moving group:{self.controlled_group_id} to {(','.join(map(str, pos.flatten())))}"
                )
        self.gui_override = True

    def drive_to_point(self, pos):
        print(f"Going to: {pos}")
        # leader_position = self.group_leader_position(self.controlled_group_id)
        # if leader_position is not None:
        #     self.leader_pos_dict.update(
        #         {self.controlled_group_id: np.append(pos, 0).reshape((-1, 1))}
        #     )
        #     if self.allow_logging:
        #         self.logger.info(
        #             f"action: moving group:{self.controlled_group_id} to {(','.join(map(str, pos.flatten())))}"
        #         )
        for ndx in self.selected_agents:
            # print(ndx)
            ndx = int(ndx)
            group_id = None
            # print(self.selected_agents)
            if ndx in self.group_manager.leaders:
                for group in self.group_manager.groups:
                    # print(self.group_manager.groups[group].agents)
                    if ndx in self.group_manager.groups[group].agents:
                        group_id = group
                # if ndx in self.group_manager.leaders:
                #     print(f"{ndx} is a LEADER")
                # else:
                #     print(f"{ndx} is not a leader")
                # group_id = self.get_group_id_from_ndx(ndx)
                if group_id is not None:
                    leader_position = self.group_leader_position(group_id)
                    if leader_position is not None:
                        self.leader_pos_dict.update(
                            {group_id: np.append(pos, 0).reshape((-1, 1))}
                        )
                        if self.allow_logging:
                            self.logger.info(
                                f"action: moving group:{group_id} to {(','.join(map(str, pos.flatten())))}"
                            )
            # print(group_id)
        self.gui_override = True

    @allow_operation
    def join_groups(self, pos):
        if self.group_manager.leaders.size > 0 and len(self.group_manager.groups) >= 2:
            ndx = self.group_manager.closest_leader_to_point(
                agent_positions=self.agent_positions, pt=pos
            )

            if ndx is not None and ndx != self._controlled_group_ndx:
                other_group_id = self.group_ids[ndx]
                group_leader_pose = self.group_leader_position(other_group_id)
                allow = in_box_area(constants.ASSEMBLY_AREA, group_leader_pose)
                if allow:
                    self.group_manager.combine(
                        main_group_id=self.controlled_group_id,
                        other_group_id=other_group_id,
                    )
                    if self.allow_logging:
                        self.logger.info(
                            f"action: Combined group {other_group_id} into {self.controlled_group_id}"
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
        print(self.group_ids[ndx], ndx)
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
