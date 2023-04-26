import time
from myrmidon.utils.plotting import create_goal_patch, create_text, modify_patch
import numpy as np
import logging


class GoalSet:
    def __init__(
        self,
        ax,
        goal_points,
        num_bots_needed,
        set_number,
        robot_positions,
        allow_logging=True,
    ) -> None:
        self.goal_points = goal_points
        self.bots_per_goal = num_bots_needed
        self.goal_radius = self.bots_per_goal * 0.15 + 0.5
        self.goal = create_goal_patch(ax, self.goal_points[0], self.goal_radius[0])
        self.goal_text = create_text(ax)
        self.check_goal = True
        self.set_number = set_number
        self.set_complete = False
        self.logger = logging.getLogger("goal_check")
        self.allow_logging = allow_logging
        self.robot_positions = robot_positions
        self.goal_num = 0
        self.wait_time = 5
        self.goal_time = None

        self.goal_text.set(
            x=self.goal_points[self.goal_num][0],
            y=self.goal_points[self.goal_num][1],
            text=self.bots_per_goal[self.goal_num],
            alpha=1,
            size=15,
        )
        self.logger.info(
            f"Start: go to goal {self.goal_num} of goal set {self.set_number}"
        )

    def goal_check(self):
        if not self.set_complete:
            num_goal_reached = self.num_in_circle()
            if (
                num_goal_reached >= self.bots_per_goal[self.goal_num]
                and self.check_goal
            ):
                self.goal_time = time.time()
                modify_patch(self.goal, facecolor="g")
                self.check_goal = False

            if not self.check_goal:
                if num_goal_reached != self.bots_per_goal[self.goal_num]:
                    self.goal_time = time.time()
                    modify_patch(self.goal, facecolor="r")
                    self.check_goal = True

                if time.time() - self.goal_time >= self.wait_time:
                    if self.goal_num < len(self.goal_points) - 1:
                        if self.allow_logging:
                            self.logger.info(
                                f"Finish: goal {self.goal_num} of goal set {self.set_number} reached"
                            )

                        self.goal_num += 1
                        modify_patch(
                            self.goal,
                            center=self.goal_points[self.goal_num],
                            radius=self.goal_radius[self.goal_num],
                            facecolor="r",
                        )
                        self.goal_text.set(
                            x=self.goal_points[self.goal_num][0],
                            y=self.goal_points[self.goal_num][1],
                            text=self.bots_per_goal[self.goal_num],
                            size=15,
                        )
                        if self.allow_logging:
                            self.logger.info(
                                f"Start: go to goal {self.goal_num} of goal set {self.set_number}"
                            )

                        self.check_goal = True
                    else:
                        if self.allow_logging:
                            self.logger.info(
                                f"Finish: goal {self.goal_num} of goal set {self.set_number} reached"
                            )

                        modify_patch(
                            self.goal,
                            radius=0,
                            facecolor="r",
                        )
                        if self.allow_logging:
                            self.logger.info(f"Goal Set {self.set_number} Completed!")
                        self.set_complete = True
                        self.goal_text.set(
                            x=0,
                            y=0,
                            text="",
                            alpha=0,
                            size=30,
                        )

    def get_circle_patch_properties(self):
        return self.goal._center, self.goal.radius

    def num_in_circle(self):
        center, radius = self.get_circle_patch_properties()
        poses = self.robot_positions.T
        count = 0
        for agent_pose in poses:
            dist = np.sqrt(
                np.power((agent_pose[0] - center[0]), 2)
                + np.power((agent_pose[1] - center[1]), 2)
            )
            if dist <= radius:
                count += 1
        return count

    def update_robot_positions(self, positions):
        self.robot_positions = positions
