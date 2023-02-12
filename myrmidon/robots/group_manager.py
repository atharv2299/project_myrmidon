import copy
from myrmidon.robots.group import Group
from myrmidon import utils
import numpy as np


def update_laplacian(func):
    def wrapper(self, *args, **kwargs):
        ret = func(self, *args, **kwargs)
        self.update_block_laplacian()
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

    def __init__(self, groups, garage):
        """
        Args:
            groups (dict[int, Group]): _description_
            garage (Group): _description_
        """
        self.groups = groups
        self.garage = garage
        self.block_L = None

    @update_laplacian
    def create(self):
        """_summary_"""

        def generate_id():
            for key, ndx in zip(list(self.groups.keys()), range(len(self.groups))):
                if key != ndx:
                    return ndx
            return ndx + 1

        new_group_id = generate_id()
        new_group = Group(
            name=GroupManager.ADJECTIVES[new_group_id // 10]
            + " "
            + GroupManager.NOUNS[new_group_id % 10],
        )
        self.groups.append(new_group)
        return new_group_id

    @update_laplacian
    def disband(self, group_id):
        """_summary_

        Args:
            group_id (_type_): _description_
        """
        group = self.groups[group_id]
        while group.agents:
            self.remove_from_group(group_id=group_id)

        del self.groups[group_id]

    @update_laplacian
    def combine(self, main_group_id, other_group_id):
        """_summary_

        Args:
            main_group (_type_): _description_
            other_group (_type_): _description_
        """
        other_group = self.groups[other_group_id]
        self.groups[main_group_id].agents.extend(other_group.agents)
        other_group.agents.clear()
        self.disband(other_group_id)

    @update_laplacian
    def split(self, group_id, num_groups):
        """_summary_

        Args:
            group_id (_type_): _description_
            num_groups (_type_): _description_
        """
        group = self.groups[group_id]
        chunks = list(utils.chunks(group.agents, num_groups))
        for agents in chunks[1:]:
            new_group_id = self.create()
            self.groups[new_group_id].agents.extend(agents)
        group.agents = chunks[0]

    @update_laplacian
    def add_to_group(self, group_id):
        """_summary_

        Args:
            group (_type_): _description_
            agent (_type_): _description_
        """
        self.groups[group_id].add(self.garage.remove())

    @update_laplacian
    def remove_from_group(self, group_id):
        """_summary_

        Args:
            group (_type_): _description_
        """
        self.garage.add(self.groups[group_id].remove())

    def update_block_laplacian(self):
        """_summary_"""
        garage = np.zeros((len(self.garage.agents), len(self.garage.agents)))
        group_list = list(self.groups.values())
        L = group_list[0].L
        agent_ids = copy.copy(group_list[0].agents)
        for group in group_list[1:]:
            formation_L = group.L
            agent_ids.extend(group.agents)
            L = np.block(
                [
                    [L, np.zeros((L.shape[0], formation_L.shape[1]))],
                    [
                        np.zeros((formation_L.shape[0], L.shape[1])),
                        formation_L,
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
        return L

    def get_dxu(self):
        """_summary_"""
        # TODO: Groups return dict, sort it, put it together

    def get_block_L(self):
        """_summary_"""
        return self.block_L

    @property
    def leaders(self):
        return np.array([group.agents[0] for group in self.groups.values()])
