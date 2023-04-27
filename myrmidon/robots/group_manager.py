import copy
import threading

import numpy as np

from myrmidon import utils
from myrmidon.robots.group import Group
from myrmidon.utils.misc import lock, time_func, in_box_area


def update_laplacian(func):
    def wrapper(self, *args, **kwargs):
        ret = func(self, *args, **kwargs)
        self._needs_laplacian_update = True
        return ret

    return wrapper


class GroupManager:
    """_summary_"""

    ADJECTIVES = [
        "angry",
        "bored",
        "cool",
        "daring",
        "elegant",
        "feral",
        "grumpy",
        "hungry",
        "invincible",
        "jovial",
    ]
    NOUNS = [
        "alligator",
        "bear",
        "cobra",
        "duck",
        "elephant",
        "fossa",
        "gorilla",
        "hippo",
        "ibex",
        "jaguar",
    ]

    def __init__(self, groups, num_agents, agent_positions):
        """
        Args:
            groups (dict[int, Group]): _description_
            garage (Group): _description_
        """
        self.groups = groups
        self.garage = Group(
            "Garage", list(range(num_agents)), positions=agent_positions
        )
        self._block_L = None
        self._needs_laplacian_update = False
        self.lock = threading.Lock()
        self.num_agents = sum(
            [len(group.agents) for group in self.groups.values()]
        ) + len(self.garage.agents)
        self.agent_positions = agent_positions

    # @update_laplacian
    def create(self, split=False):
        """_summary_"""

        if not split and not self.garage.agents:
            return
        agent = self.garage.remove()
        if agent is None:
            return
        self.garage.add(agent)

        def generate_id():
            ndx = -1
            for key, ndx in zip(list(self.groups.keys()), range(len(self.groups))):
                if key != ndx and ndx not in list(self.groups.keys()):
                    return ndx
            return ndx + 1

        new_group_id = generate_id()

        new_group = Group(
            name=GroupManager.ADJECTIVES[new_group_id // 10]
            + " "
            + GroupManager.NOUNS[new_group_id % 10],
        )
        self.groups[new_group_id] = new_group
        return new_group_id

    @update_laplacian
    def disband(self, group_id):
        """_summary_

        Args:
            group_id (_type_): _description_
        """
        if group_id not in self.groups:
            return
        group = self.groups[group_id]
        while group.agents:
            self.remove_from_group(group_id=group_id)
        if group_id in self.groups:
            del self.groups[group_id]

    @lock
    @update_laplacian
    def combine(self, main_group_id, other_group_id):
        """_summary_

        Args:
            main_group (_type_): _description_
            other_group (_type_): _description_
        """
        other_group_leader = self.groups[other_group_id].agents[0]
        other_group_leader_pose = self.agent_positions[:, [other_group_leader]]
        if main_group_id == other_group_id or not in_box_area(
            utils.constants.ASSEMBLY_AREA, other_group_leader_pose
        ):
            return
        other_group = self.groups[other_group_id]
        self.groups[main_group_id].extend(other_group.agents)
        other_group.clear()
        self.disband(other_group_id)

    @lock
    @update_laplacian
    def split(self, group_id, num_groups):
        """_summary_

        Args:
            group_id (_type_): _description_
            num_groups (_type_): _description_
        """
        if group_id not in self.groups:
            return
        group = self.groups[group_id]
        # TODO: Check this works
        # split_groups = np.array_split(group.agents, num_groups)
        split_groups = list(utils.chunks(group.agents, num_groups))
        for agents in split_groups[1:]:
            if agents:
                new_group_id = self.create(True)
                self.groups[new_group_id].extend(agents)
        group.clear()
        group.extend(split_groups[0])

    @lock
    @update_laplacian
    def add_to_group(self, group_id):
        """_summary_

        Args:
            group (_type_): _description_
            agent (_type_): _description_
        """
        if not self.garage.agents:
            return
        agent = self.garage.remove()
        if agent is None:
            return
        self.groups[group_id].add(agent)

    @lock
    @update_laplacian
    def remove_from_group(self, group_id):
        """_summary_

        Args:
            group (_type_): _description_
        """
        self.garage.add(self.groups[group_id].remove())
        if not self.groups[group_id].agents:
            self.disband(group_id)

    @lock
    @update_laplacian
    def change_group_graph(self, group_id, graph):
        self.groups[group_id].set_graph(graph)

    @lock
    def get_dxu(
        self,
        leader_dxus,
        garage_locs,
        agent_positions,
        garage_controller,
        leader_position_controller,
        si_to_uni_dyn,
        uni_to_si_dyn,
        uni_barrier_certs,
        desired_leader_position,
        walls=None,
    ):
        """_summary_"""

        def garage_dxu():
            dxu_dict = {}
            for agent in self.garage.agents:
                dxu_dict[agent] = garage_controller(
                    agent_positions[:, [agent]],
                    garage_locs[:, [agent]],
                )
            return dxu_dict

        dxu_dict = {}
        dxu_dict.update(garage_dxu())
        for group_id, group in self.groups.items():
            if group_id in desired_leader_position:
                gui_leader_dxu = leader_position_controller(
                    agent_positions[:, [self.groups[group_id].agents[0]]],
                    desired_leader_position.get(group_id),
                )
                if (
                    np.linalg.norm(
                        agent_positions[:2, [self.groups[group_id].agents[0]]]
                        - desired_leader_position.get(group_id)[:2]
                    )
                    <= utils.constants.CLOSE_ENOUGH
                ):
                    desired_leader_position.pop(group_id)
            else:
                gui_leader_dxu = np.array([[0], [0]])
            leader_dxu = leader_dxus.get(group_id, gui_leader_dxu)
            group_dxu = group.calculate_follower_dxus(
                agent_positions, leader_dxu, si_to_uni_dyn, uni_to_si_dyn, walls
            )
            dxu_dict.update(group_dxu)

        dxu = np.zeros((2, self.num_agents))
        for agent_id in sorted(dxu_dict):
            dxu[:, [agent_id]] = dxu_dict[agent_id]

        dxu = uni_barrier_certs(dxu, agent_positions)
        return dxu

    def closest_leader_to_point(self, agent_positions, pt):
        leader_positions = agent_positions[:2, self.leaders] - np.array([[0.12], [0]])
        dists = np.linalg.norm(leader_positions - pt, axis=0)
        dists_in_range = dists[dists < utils.constants.LEADER_SELECTION_RADIUS]
        if not dists_in_range.size > 0:
            return
        closest_leader_ndx = np.argmin(dists)
        return closest_leader_ndx

    def update_positions(self, positions):
        # For older versions of python
        self.garage.update_positions(positions)
        self.agent_positions = positions

    @property
    def leaders(self):
        return np.array([group.agents[0] for group in self.groups.values()])

    @property
    def followers(self):
        return np.array(
            [
                np.array(group.agents[1:])
                for group in self.groups.values()
                if len(group.agents) > 1
            ]
        ).reshape((-1))

    @property
    def block_L(self):
        def update_block_laplacian():
            """_summary_"""
            garage = np.zeros((len(self.garage.agents), len(self.garage.agents)))
            group_list = list(self.groups.values())
            if not group_list:
                self._block_L = garage
                return
            L = group_list[0].L
            agent_ids = copy.copy(group_list[0].agents)
            for group in group_list[1:]:
                agent_ids.extend(group.agents)
                L = np.block(
                    [
                        [L, np.zeros((L.shape[0], group.L.shape[1]))],
                        [
                            np.zeros((group.L.shape[0], L.shape[1])),
                            group.L,
                        ],
                    ]
                )
            L = np.block(
                [
                    [L, np.zeros((L.shape[0], garage.shape[1]))],
                    [
                        np.zeros((garage.shape[0], L.shape[1])),
                        garage,
                    ],
                ]
            )
            full_list = list(agent_ids) + list(self.garage.agents)
            full_list = np.argsort(full_list)
            L = -L[full_list, :][:, full_list]
            self._block_L = L

        if self._needs_laplacian_update:
            update_block_laplacian()
            self._needs_laplacian_update = False

        return self._block_L
