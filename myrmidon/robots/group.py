import numpy as np
from myrmidon import utils
from myrmidon.utils.graph import Graphs

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
        self.graph = Graphs.MINIMAL

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

    def calculate_follower_dxus(self, poses, leader_dxu, si_to_uni_dyn):
        """BARRIERLESS DXU

        Args:
            poses (_type_): _description_
            leader_dxu (_type_): _description_
            si_to_uni_dyn (_type_): _description_

        Returns:
            dict[np.ndarray[double]]: N barrierless 2x1 unicycle dynamic control for agents in this formation, including the leader
        """
        if not self.agents:
            return {}

        dxs = np.zeros((2, len(self.agents)))
        dxu = {}
        L = self.L.copy()
        for ndx, agent_id in enumerate(self.agents):
            neighbors = utils.graph.topological_neighbors(L, ndx)
            for neighbor_ndx in neighbors:
                neighbor = self.agents[neighbor_ndx]
                dxs[:, [ndx]] += (
                    self.control_gain
                    * (
                        np.power(
                            np.linalg.norm(
                                poses[:2, [neighbor]] - poses[:2, [agent_id]]
                            ),
                            2,
                        )
                        - np.power(self.dist_scale * self.dists[ndx, neighbor_ndx], 2)
                    )
                    * (poses[:2, [neighbor]] - poses[:2, [agent_id]])
                )
                dxu[agent_id] = si_to_uni_dyn(dxs[:, [ndx]], poses[:, [agent_id]])
        # TODO: Scale leader dxu based on distance to connected followers
        # TODO: Add unique leader barriers
        # leader_barrier = unicycle_connectivity_barriers(
        #     connectivity_radius=self.dist_scale * 1.1
        # )
        # barriered_dxu = leader_barrier(leader_dxu, poses[:, [self.agents[0]]])
        # print(leader_dxu)
        # print(barriered_dxu)
        # barriered_dxu = leader_dxu
        # leader_follower_ndx = np.array(list(set(utils.misc.find_connections(-L)[0])))
        # leader_ndx = 0
        # leader_positon = positions[:2, [leader_ndx]]
        # leader_follower_ndx = utils.graph.topological_neighbors(L, leader_ndx)
        # if leader_follower_ndx:
        #     leader_follower_ids = self.agents[leader_follower_ndx]
        #     leader_follower_positions = positions[:2, [leader_follower_ids]]
        #     actual_follower_dists = np.linalg.norm(leader_positon - leader_follower_positions)
        #     desired_follower_dists = self.dist_scale * self.dists[leader_ndx, leader_follower_ndx]

        # scaling_factor = 1 or
        # # print(positions)
        # print(leader_follower_positions)
        dxu[self.agents[0]] = leader_dxu
        return dxu

    @update_laplacian
    def set_graph(self, graph):
        self.graph = graph

    @property
    def L(self):
        if self._needs_laplacian_update:
            if not self.agents:
                self._L = self.dists = None
            else:
                self._L, self.dists = self.create_graph(len(self.agents))
            self._needs_laplacian_update = False
        return self._L

    @property
    def create_graph(self):
        if self.graph == Graphs.MINIMAL:
            return utils.graph.rigid_minimal_GL()
        elif self.graph == Graphs.CYCLE:
            return utils.graph.cycle_GL()
        elif self.graph == Graphs.COMPLETE:
            return utils.graph.complete_GL()
        elif self.graph == Graphs.LINE:
            return utils.graph.line_GL()
