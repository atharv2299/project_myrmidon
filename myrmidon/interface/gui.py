import matplotlib
import numpy as np

matplotlib.use("TkAgg")
from tkinter import *
from tkinter.ttk import *
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from functools import partial
import matplotlib.pyplot as plt


class GUI:
    def __init__(
        self, root, group_manager, robotarium_figure, leader_controller, agent_positions
    ):
        self.root = root
        self.group_manager = group_manager
        self.robotarium_figure = robotarium_figure
        self.leader_controller = leader_controller
        self.root.title("Myrmidon")
        self.root.geometry("1200x600")
        # self._selected_group_ndx = [0, None]
        # self._selection_ndx_marker = 0
        # self._second_group_ndx = None
        self.leader_selection_flag = True
        self.go_to_point_flag = False
        self.leader_pos = {}
        self._controlled_group_ndx = 0
        self.agent_positions = agent_positions

        # Iniitialize for Robotarium
        self.fig = Figure(figsize=(5, 5), dpi=100, facecolor="w")
        self.fig = self.robotarium_figure
        plt.close(robotarium_figure)
        self.selector = self.fig.canvas.mpl_connect(
            "button_press_event", self.selection
        )
        cid = self.fig.canvas.mpl_connect("button_press_event", self.onclick)

        # Initialize Matplotlib features
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        self.canvas.get_tk_widget().place(x=20, y=30)

        # Display Buttons, Sliders
        button_select = Button(
            self.root, text="Select", command=self.button_select
        ).place(x=800, y=30)
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

        # self.scale1 = Scale(
        #     self.root, variable=v1, from_=0.5, to=1.0, orient=HORIZONTAL
        # ).place(x=800, y=390)
        # scale1 = CTk.CTkSlider(master=self.root, from_=0, to=100, variable=v1)
        # self.label_scale1 = Label(
        #     self.root, text=" Distance: " + str(self.get_dist)
        # ).place(x=910, y=390)

        button_go_to_point = Button(
            self.root, text="Go To Point", command=self.go_to_point
        ).place(x=800, y=480)

        # TODO: Check select_groups() for reference
        # self.selected_groups = np.array()

    # # TODO: Check if I did this even close to right
    # def select_groups(self, group_id):
    #     # selected_groups = np.array()
    #     self.selected_groups.append(group_id)
    #     self.selected_groups = np.unique(self.selected_groups)
    #     return self.selected_groups

    # def clear_select(self):
    #     self.selected_groups = np.array()

    # Define button functions
    # group_manager.function_call()
    def button_join(self):
        print("Aggregate function plz")
        # group_manager.combine(
        #     main_group_id=selected_groups[-2],
        #     other_group_id=selected_groups[-1]
        # )
        # self.group_manager.clear_select()

    def button_separate(self):
        print("Separate function plz")
        self.group_manager.split(group_id=self.controlled_group_id, num_groups=2)

    def button_newleader(self):
        print("New Leader function plz")
        group_id = self.group_manager.create()
        self.group_manager.add_to_group(group_id)

    def button_disband(self):
        print("Disband function plz")
        self.group_manager.disband(group_id=self.controlled_group_id)
        # self.group_manager.clear_select()

    def add_robot(self):
        print("Add Robot")
        self.group_manager.add_to_group(group_id=self.controlled_group_id)
        # self.group_manager.clear_select()

    def remove_robot(self):
        print("Remove Robot")
        self.group_manager.remove_from_group(group_id=self.controlled_group_id)
        # self.group_manager.clear_select()

    def next_controlled_group(self):
        self._controlled_group_ndx = (self._controlled_group_ndx + 1) % (
            len(self.group_ids)
        )

    def button_select(self):
        self.leader_selection_flag = True
        self.fig.canvas.mpl_connect("button_press_event", self.selection)

    def selection(self, event):
        # print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
        #       ('double' if event.dblclick else 'single', event.button,
        #        event.x, event.y, event.xdata, event.ydata))
        # TODO: closest leader to point takes all agent positions and then finds the leaders from there
        if self.leader_selection_flag:
            pos = np.array([[event.xdata], [event.ydata]])
            print(pos)
            if self.group_manager.leaders.size > 0:
                self._controlled_group_ndx = self.group_manager.closest_leader_to_point(
                    agent_positions=self.agent_positions,
                    pt=pos,
                    ndx=self._controlled_group_ndx,
                )
            # self.leader_selection_flag = False

    def onclick(self, event):
        # print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
        #       ('double' if event.dblclick else 'single', event.button,
        #        event.x, event.y, event.xdata, event.ydata))
        # TODO: closest leader to point takes all agent positions and then finds the leaders from there
        if self.go_to_point_flag:
            pos = np.array([[event.xdata], [event.ydata]])
            print(f"Going to: {pos}")
            leader_pos = self.group_leader_position(self.controlled_group_id)
            if leader_pos is not None:
                self.leader_pos.update({self.controlled_group_id: pos})
            self.go_to_point_flag = False
            self.leader_selection_flag = True

    def go_to_point(self):
        # set Flag to True
        # gtg_flag = True
        self.leader_selection_flag = False
        self.go_to_point_flag = True
        print("Go To Point")

        # while True:
        #     gtg_pos = np.array([[cid.x], [cid.y]])
        # Push gtg_pos into a dxu_controller for go_to_point with selected_groups[-1]
        # gtg_flag = False

    # TODO: Move this, doesn't need to be in the group_manager, gui has access to this info
    def group_leader_position(self, group_id):
        if group_id not in self.group_manager.groups:
            return
        leader_idx = self.group_manager.groups[group_id].agents[0]
        return self.agent_positions[:, [leader_idx]]

    def update_gui(self):
        self.canvas.draw()
        self.toolbar.update()
        # print(c.groups.formations)
        # print(c.groups.controlled_formation)
        # c.drive_robots()

    def get_group_id_from_ndx(self, ndx):
        if ndx >= len(self.group_ids):
            return None
        return self.group_ids[ndx]

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
