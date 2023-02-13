import numpy as np
from pynput import keyboard


class TUI:
    def __init__(self, group_manager) -> None:
        self.v = 0
        self.w = 0
        self.group_manager = group_manager
        self.controlled_formation = 0
        self.leader_dxu = {}
        # TODO: Move to application
        # listener = keyboard.Listener(on_press=self.on_press, suppress=True)
        # listener.start()

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
                self.group_manager.add_to_group(
                    self.group_ids[self.controlled_formation]
                )
                is_bot_count_updated = True

            elif pressed_key.char == "c":
                self.group_manager.remove_from_group(
                    self.group_ids[self.controlled_formation]
                )
                is_bot_count_updated = True

            # Change desired spacing between bots
            elif pressed_key.char == "o":
                new_scale = min(
                    self.group_manager.groups[
                        self.group_ids[self.controlled_formation]
                    ].dist_scale
                    + 0.05,
                    0.7,
                )
                self.group_manager.groups[
                    self.group_ids[self.controlled_formation]
                ].set_dist_scale(new_scale)
                print(
                    f"=============================================\nRobots are now maintaining a distance of {new_scale:.2f}\n============================================="
                )

            elif pressed_key.char == "l":
                new_scale = max(
                    self.group_manager.groups[
                        self.group_ids[self.controlled_formation]
                    ].dist_scale
                    - 0.05,
                    0.3,
                )
                self.group_manager.groups[
                    self.group_ids[self.controlled_formation]
                ].set_dist_scale(new_scale)
                print(
                    f"=============================================\nRobots are now maintaining a distance of {new_scale:.2f}\n============================================="
                )

            elif pressed_key.char == "r":
                self.group_manager.split(self.group_ids[self.controlled_formation], 2)

            elif pressed_key.char == "t":
                if len(self.group_manager.groups) >= 2:
                    self.group_manager.combine(
                        self.group_ids[0],
                        self.group_ids[-1],
                    )

            self.leader_dxu = {
                self.group_ids[self.controlled_formation]: np.tile(
                    np.array([[self.v], [self.w]]), (1, 1)
                )
            }

        except AttributeError:
            if pressed_key == keyboard.Key.space:
                print(self.group_manager.groups)
            elif pressed_key == keyboard.Key.tab and len(self.group_manager.groups) > 1:
                self.next_controlled_formation()

                print(
                    f"=========================================\nControlling formation: {self.controlled_formation}\nFormations: {self.group_manager.groups}\n ========================================="
                )
        if is_bot_count_updated:
            print(
                f"========================================= \nThere are now {len(self.group_manager.groups[self.group_ids[self.controlled_formation]].agents)} robot(s) in formation #{self.group_manager.groups[self.group_ids[self.controlled_formation]]} \nwhich are mantaining a distance of {self.group_manager.groups[self.group_ids[self.controlled_formation]].dist_scale:.2f}\n========================================="
            )
        print(f"Linear: {self.v:.3f}   |    Angular: {self.w:.2f}")

    def next_controlled_formation(self):
        self.controlled_formation = self.group_ids[
            (self.controlled_formation + 1) % (len(self.group_ids))
        ]

    @property
    def group_ids(self):
        return list(self.group_manager.groups.keys())
