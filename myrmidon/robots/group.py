import numpy as np
from myrmidon import utils

# TODO: ALL DOCUMENTATION


def update_laplacian(func):
    def wrapper(self, *args, **kwargs):
        ret = func(self, *args, **kwargs)
        self._needs_laplacian_update = True
        return ret

    return wrapper


class Group:
    def __init__(self, name, agents=None):
        self.agents = agents or []
        self.name = name
        self.control_gain = 0.4
        self.dist_scale = 0.35
        self._L = None
        self.dists = None
        self._needs_laplacian_update = False

    @update_laplacian
    def add(self, agent_id):
        self.agents.append(agent_id)

    @update_laplacian
    def extend(self, agent_ids):
        self.agents.extend(agent_ids)

    @update_laplacian
    def remove(self):
        return self.agents.pop()

    @update_laplacian
    def clear(self):
        self.agents.clear()

    def set_dist_scale(self, new_dist_scale):
        self.dist_scale = new_dist_scale

    def calculate_follower_dxus(self, positions, leader_dxu, si_to_uni_dyn):
        """BARRIERLESS DXU

        Args:
            positions (_type_): _description_
            leader_dxu (_type_): _description_
            si_to_uni_dyn (_type_): _description_

        Returns:
            dict[np.ndarray[double]]: N barrierless 2x1 unicycle dynamic control for agents in this formation, including the leader
        """
        if not self.agents:
            return {}

        dxs = np.zeros((2, len(self.agents)))
        dxu = {}
        for ndx, agent_id in enumerate(self.agents):
            L = self.L.copy()
            neighbors = utils.graph.topological_neighbors(L, ndx)
            for neighbor_ndx in neighbors:
                neighbor = self.agents[neighbor_ndx]
                dxs[:, [ndx]] += (
                    self.control_gain
                    * (
                        np.power(
                            np.linalg.norm(
                                positions[:2, [neighbor]] - positions[:2, [agent_id]]
                            ),
                            2,
                        )
                        - np.power(self.dist_scale * self.dists[ndx, neighbor_ndx], 2)
                    )
                    * (positions[:2, [neighbor]] - positions[:2, [agent_id]])
                )
                dxu[agent_id] = si_to_uni_dyn(dxs[:, [ndx]], positions[:, [agent_id]])
        # TODO: Scale leader dxu based on distance to connected followers
        # leader_follower_ndx = np.array(list(set(utils.misc.find_connections(-L)[0])))
        # leader_followers = self.agents[leader_follower_ndx]
        # leader_follower_positions = positions[:2, leader_followers]
        # # print(positions)
        # print(leader_follower_positions)
        dxu[self.agents[0]] = leader_dxu
        return dxu

    @property
    def L(self):
        if self._needs_laplacian_update:
            if not self.agents:
                self._L = self.dists = None
            else:
                self._L, self.dists = utils.graph.rigid_cycle_GL(len(self.agents))
            self._needs_laplacian_update = False
        return self._L
