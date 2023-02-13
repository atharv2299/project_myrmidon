import numpy as np
from pynput import keyboard


class TUI:
    def __init__(self, group_manager, allow_multi_leader_movement=False) -> None:
        self.v = 0
        self.w = 0
        self.group_manager = group_manager
        self._controlled_group_ndx = 0
        self.leader_dxu = {}
        self.exit = False
        self.allow_multi_leader_movement = allow_multi_leader_movement

    def on_press(self, key):
        self.key_function(key)

    def key_function(self, pressed_key):
        is_bot_count_updated = False
        angular_increment = 0.1
        linear_increment = 0.015

        try:
            # Linear and angular control
            if pressed_key.char == "h":
                group_id = self.group_manager.create()
                self.group_manager.add_to_group(group_id)
            elif not self.group_manager.groups:
                return
            if pressed_key.char == "w":
                self.v += linear_increment
            elif pressed_key.char == "a":
                self.w += angular_increment
            elif pressed_key.char == "x":
                self.v -= linear_increment
            elif pressed_key.char == "d":
                self.w -= angular_increment
            elif pressed_key.char == "s":
                self.v = 0
                self.w = 0

            # Stop linear and increment angular
            elif pressed_key.char == "q":
                self.v = 0
                self.w += angular_increment
            elif pressed_key.char == "e":
                self.v = 0
                self.w -= angular_increment

            # Add and remove individual robots
            elif pressed_key.char == "z":
                self.group_manager.add_to_group(self.controlled_group_id)
                is_bot_count_updated = True

            elif pressed_key.char == "c":
                self.group_manager.remove_from_group(self.controlled_group_id)
                is_bot_count_updated = True

            # Change desired spacing between bots
            elif pressed_key.char == "o":
                new_scale = min(
                    self.controlled_group.dist_scale + 0.05,
                    0.7,
                )
                self.controlled_group.set_dist_scale(new_scale)
                print(
                    f"=============================================\nRobots are now maintaining a distance of {new_scale:.2f}\n============================================="
                )

            elif pressed_key.char == "l":
                new_scale = max(
                    self.controlled_group.dist_scale - 0.05,
                    0.3,
                )
                self.controlled_group.set_dist_scale(new_scale)
                print(
                    f"=============================================\nRobots are now maintaining a distance of {new_scale:.2f}\n============================================="
                )

            elif pressed_key.char == "r":
                self.group_manager.split(self.controlled_group_id, 2)

            elif pressed_key.char == "t":
                if len(self.group_manager.groups) >= 2:
                    # if self._controlled_group_ndx == self.group_ids[-1]:
                    #     self._controlled_group_ndx = self.group_ids[0]

                    self.group_manager.combine(
                        self.group_ids[0],
                        self.group_ids[-1],
                    )
            elif pressed_key.char == "y":
                self.group_manager.disband(self.controlled_group_id)

            # TODO: Check this
            # Find the next possible controllable group if the current group has been invalidated.
            if len(self.group_ids):
                while self.controlled_group is None:
                    self.next_controlled_group()

            if not self.group_manager.groups:
                return
            # Be able to move multiple leaders at once:
            if self.allow_multi_leader_movement:
                self.leader_dxu.update(
                    {
                        self.controlled_group_id: np.tile(
                            np.array([[self.v], [self.w]]), (1, 1)
                        )
                    }
                )
            # Can only move single leader:
            else:
                self.leader_dxu = {
                    self.controlled_group_id: np.tile(
                        np.array([[self.v], [self.w]]), (1, 1)
                    )
                }

        except AttributeError:
            if pressed_key == keyboard.Key.space:
                print(self.group_manager.groups)
            elif pressed_key == keyboard.Key.tab and len(self.group_manager.groups) > 1:
                self.next_controlled_group()

                print(
                    f"=========================================\nControlling formation: {self.controlled_group_id}\nFormations: {self.group_manager.groups}\n ========================================="
                )
            elif pressed_key == keyboard.Key.esc:
                self.exit = True
        if is_bot_count_updated:
            print(
                f"========================================= \nThere are now {len(self.controlled_group.agents)} robot(s) in formation #{self.controlled_group} \nwhich are mantaining a distance of {self.controlled_group.dist_scale:.2f}\n========================================="
            )
        print(f"Linear: {self.v:.3f}   |    Angular: {self.w:.2f}")

    def next_controlled_group(self):
        self._controlled_group_ndx = (self._controlled_group_ndx + 1) % (
            len(self.group_ids)
        )
        self.v = 0
        self.w = 0

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
