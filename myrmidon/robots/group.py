import numpy as np
from myrmidon import utils
from myrmidon.utils import constants

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
        # TODO: Change control gain for real robots
        self.control_gain = 2
        self.dist_scale = 0.5
        self._L = None
        self.dists = None
        self._needs_laplacian_update = False
        self.graph = "Rigid Minimal"

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
        if new_dist_scale <= constants.MAX_DIST_SCALE:
            self.dist_scale = new_dist_scale

    def calculate_follower_dxus(
        self, poses, leader_dxu, si_to_uni_dyn, uni_to_si_dyn, walls=None
    ):
        """BARRIERLESS DXU

        Args:
            poses (_type_): _description_
            leader_dxu (_type_): _description_
            si_to_uni_dyn (_type_): _description_

        Returns:
            dict[np.ndarray[double]]: N barrierless 2x1 unicycle dynamic control for agents in this formation, including the leader
        """
        if self.graph == "Directed Cycle":
            dxu = self.cyclic_pursuit_control(
                poses, si_to_uni_dyn, uni_to_si_dyn, walls
            )
        else:
            dxu = self.leader_follower_control(
                poses, leader_dxu, si_to_uni_dyn, uni_to_si_dyn, walls
            )

        return dxu

    def leader_follower_control(
        self, poses, leader_dxu, si_to_uni_dyn, uni_to_si_dyn, walls=None
    ):
        if not self.agents:
            return {}
        leader_dxs = uni_to_si_dyn(leader_dxu, poses[:, [self.agents[0]]])
        leader_dxs = self.wall_avoidance(walls, poses, self.agents[0], leader_dxs)
        leader_dxu = si_to_uni_dyn(leader_dxs, poses[:, [self.agents[0]]])

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
                # TODO: Finish implementation
                if walls is not None:
                    dxs[:, [ndx]] = self.wall_avoidance(
                        walls, poses, agent_id, dxs[:, [ndx]]
                    )
                dxu[agent_id] = si_to_uni_dyn(dxs[:, [ndx]], poses[:, [agent_id]])
        dxu[self.agents[0]] = leader_dxu
        return dxu

    def cyclic_pursuit_control(self, poses, si_to_uni_dyn, uni_to_si_dyn, walls=None):
        if not self.agents:
            return {}

        dxs = np.zeros((2, len(self.agents)))
        dxu = {}
        L = self.L.copy()
        theta_offset = np.pi / len(self.agents) * self.dist_scale
        for ndx, agent_id in enumerate(self.agents):
            neighbors = utils.graph.topological_neighbors(L, ndx)
            for neighbor_ndx in neighbors:
                neighbor = self.agents[neighbor_ndx]
                delta = poses[:2, [neighbor]] - poses[:2, [agent_id]]
                agent_angle = np.arctan2(delta[1], delta[0])
                angle = agent_angle - theta_offset
                magnitude = np.linalg.norm(delta)
                dxs[:, [ndx]] += (
                    magnitude
                    * np.array([[np.cos(angle)], [np.sin(angle)]]).reshape((2, -1))
                    + self.control_gain * delta
                )
                dxu[agent_id] = si_to_uni_dyn(dxs[:, [ndx]], poses[:, [agent_id]])
        return dxu

    def wall_avoidance(self, walls, poses, agent_id, dxs):
        norm_dxs = dxs / np.linalg.norm(dxs)
        too_close = False
        pose = poses[:2, [agent_id]]
        line = np.array(
            [
                np.reshape(pose, (1, 2)),
                np.reshape(
                    norm_dxs * constants.WALL_PROJECTION_DIST + pose,
                    (1, 2),
                ),
            ]
        ).reshape(2, 2)

        for wall in walls:
            # wall_vector = (wall[0] - wall[1]).reshape((2,))
            # agent_vector = (pose.reshape(wall[1].shape) - wall[1]).reshape((2,))

            # if np.dot(wall_vector, agent_vector) > 0:
            #     theta = np.arccos(
            #         np.dot(
            #             wall_vector / np.linalg.norm(wall_vector),
            #             agent_vector / np.linalg.norm(agent_vector),
            #         )
            #     )
            #     dist = np.linalg.norm(agent_vector) * np.sin(theta)
            #     if dist <= constants.WALL_PROJECTION_DIST / 2:
            #         too_close = True

            M = np.array([line[1] - line[0], wall[0] - wall[1]]).T
            singular = np.linalg.det(M) == 0
            if not singular:
                t, s = np.linalg.solve(M, wall[0] - line[0])
                if (0 <= t <= 1) and (0 <= s <= 1):
                    new_dxs = np.zeros((2, 1))
                    # if too_close:
                    #     new_dxs = -norm_dxs / 10
                    return new_dxs
        return dxs

    @update_laplacian
    def set_graph(self, graph):
        if graph in constants.GRAPHS:
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
        if self.graph == "Rigid Minimal":
            return utils.graph.rigid_minimal_GL()
        elif self.graph == "Cycle":
            return utils.graph.cycle_GL()
        elif self.graph == "Complete":
            return utils.graph.complete_GL()
        elif self.graph == "Line":
            return utils.graph.line_GL()
        elif self.graph == "Directed Cycle":
            return utils.graph.directed_cycle_GL()
