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
    def __init__(self, root, group_manager, robotarium_figure, leader_controller):
        self.root = root
        self.group_manager = group_manager
        self.robotarium_figure = robotarium_figure
        self.leader_controller = leader_controller
        self.root.title("Myrmidon")
        self.root.geometry("1200x600")
        self._controlled_group_ndx = 0

        # Iniitialize for Robotarium
        self.fig = Figure(figsize=(5, 5), dpi=100, facecolor="w")
        self.fig = self.robotarium_figure
        plt.close(robotarium_figure)
        # Initialize Matplotlib features
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        # canvas.get_tk_widget().pack(side="left")
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.root)
        # canvas.get_tk_widget().pack(side="left")
        self.canvas.get_tk_widget().place(x=20, y=30)

        # Display Buttons, Sliders
        button11 = Button(self.root, text="Select", command=self.root.destroy).place(
            x=800, y=30
        )
        button12 = Button(self.root, text="Unselect", command=self.root.destroy).place(
            x=900, y=30
        )

        button_newleader = Button(
            self.root, text="New Leader", command=self.button_join, state=NORMAL
        ).place(x=800, y=120)

        button2 = Button(
            self.root, text="Join", command=self.button_join, state=NORMAL
        ).place(x=1000, y=120)

        button_disband = Button(
            self.root, text="Disband Group", command=self.button_disband, state=NORMAL
        ).place(x=800, y=210)

        button3 = Button(
            self.root, text="Separate", command=self.button_separate
        ).place(x=1000, y=210)

        button4 = Button(self.root, text="+", width=3, command=self.add_robot).place(
            x=800, y=300
        )  # command=c.groups.add_agent

        entry4 = Entry(self.root, width=5).place(x=867, y=300)

        button4 = Button(self.root, text="-", width=3, command=self.remove_robot).place(
            x=925, y=300
        )  # command=partial(c.groups.split_network, c.L, c.dists)

        # self.scale1 = Scale(
        #     self.root, variable=v1, from_=0.5, to=1.0, orient=HORIZONTAL
        # ).place(x=800, y=390)
        # scale1 = CTk.CTkSlider(master=self.root, from_=0, to=100, variable=v1)
        # self.label_scale1 = Label(
        #     self.root, text=" Distance: " + str(self.get_dist)
        # ).place(x=910, y=390)

        self.button5 = Button(
            self.root, text="Go To Point", command=self.go_to_point
        ).place(x=800, y=480)

    # Define button functions
    # group_manager.fucntion_call()
    def button_join(self):
        print("Aggregate function plz")

    def button_separate(self):
        print("Separate function plz")

    def button_newleader(self):
        print("New Leader function plz")

    def button_disband(self):
        print("Disband function plz")

    def add_robot(self):
        print("Add Robot")
        # c.groups.add_agent()

    def remove_robot(self):
        print("Remove Robot")
        # c.groups.split_network(c.L, c.dists)

    def todo():
        pass

    def onclick(self, event):
        # print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
        #       ('double' if event.dblclick else 'single', event.button,
        #        event.x, event.y, event.xdata, event.ydata))
        pos = np.array([[event.x], [event.y]])
        # Sends to group_manager.closest_leader -> group_id Leader Selection
        # Feeds point into leader_dxu = {group_id: (self.leader_controller(leader_position, pt))}
        # Should probably use leader_dxu.update
        print(pos)
        return pos

    def go_to_point(self):
        print("Go To Point")
        # cid = fig.canvas.mpl_connect("button_press_event", self.onclick)

    def update_gui(self):
        self.canvas.draw()
        self.toolbar.update()
        # print(c.groups.formations)
        # print(c.groups.controlled_formation)
        # c.drive_robots()

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
