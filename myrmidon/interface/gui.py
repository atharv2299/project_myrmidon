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
        self._controlled_group_ndx = 0
        self.agent_positions = agent_positions

        # TODO: Need way to access self.selected_groups from group_manager
        # The GUI is going to track selected groups, group_manager will not know what groups a person has selected, the gui will tell it what to do and with what group

        # Iniitialize for Robotarium
        self.fig = Figure(figsize=(5, 5), dpi=100, facecolor="w")
        self.fig = self.robotarium_figure
        plt.close(robotarium_figure)

        # Initialize Matplotlib features
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        self.canvas.get_tk_widget().place(x=20, y=30)

        # Display Buttons, Sliders
        button_select = Button(
            self.root, text="Select", command=self.root.destroy
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
            self.root, text="Go To Point", command=self.onclick
        ).place(x=800, y=480)

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
        # self.group_manager.split(
        #     group_id=selected_groups[-1],
        #     num_groups=2
        # )
        # self.group_manager.clear_select()

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

    def onclick(self, event):
        # print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
        #       ('double' if event.dblclick else 'single', event.button,
        #        event.x, event.y, event.xdata, event.ydata))
        # TODO: closest leader to point takes all agent positions and then finds the leaders from there
        pos = np.array([[event.x], [event.y]])
        if self.group_manager.leaders.size > 0:
            leader_ndx = self.group_manager.closest_leader_to_point(
                agent_positions=self.agent_positions, pt=pos
            )
            group_id = self.get_group_id_from_ndx(leader_ndx)

        # selected_groups = group_manager.select_groups(group_id)

        # Sends to group_manager.closest_leader -> group_id Leader Selection
        # Feeds point into leader_dxu = {group_id: (self.leader_controller(leader_position, pt))}
        # Should probably use leader_dxu.update
        print(pos)
        return pos

    def go_to_point(self, event):
        # set Flag to True
        gtg_flag = True
        print("Go To Point")
        # cid = fig.canvas.mpl_connect("button_press_event", self.onclick)
        while True:
            gtg_pos = np.array([[event.x], [event.y]])
            # Push gtg_pos into a dxu_controller for go_to_point with selected_groups[-1]
        gtg_flag = False

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
