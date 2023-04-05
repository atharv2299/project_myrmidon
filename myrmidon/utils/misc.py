import math
import time

import numpy as np
from cvxopt import matrix, sparse
from cvxopt.blas import dot
from cvxopt.solvers import options, qp
from rps.utilities.transformations import *
from scipy.special import comb
from myrmidon.utils.graph import topological_neighbors
import threading


# TODO: Replace with numpy array_split
def chunks(seq, num_chunks):
    chunk_size = int(math.ceil(len(seq) / num_chunks))
    for i in range(num_chunks):
        yield seq[i * chunk_size : (i + 1) * chunk_size]


def find_connections(L):
    """
    Takes in a graph Laplacian and returns leaders and agents connected to leaders
    """
    [rows, cols] = np.where(L == -1)
    return rows, cols


def unique_list(lst):
    ret = []
    seen = set()
    for elem in lst:
        if elem not in seen:
            ret.append(elem)
        seen.add(elem)

    return ret


def time_func(func):
    def wrapper(*args, **kwargs):
        start = time.time()
        ret = func(*args, **kwargs)
        stop = time.time()
        print(f"{func.__name__}: {stop-start} seconds")
        return ret

    return wrapper


def lock(func):
    def wrapper(self, *args, **kwargs):
        self.lock.acquire()
        ret = func(self, *args, **kwargs)
        self.lock.release()
        return ret

    return wrapper


# TODO: Add custom barrier functions here:
def custom_uni_barriers(
    barrier_gain=100,
    safety_radius=0.12,
    projection_distance=0.05,
    magnitude_limit=0.2,
    boundary_points=np.array([-1.6, 1.6, -1.0, 1.0]),
    connectivity_distance=0.5,
    group_manager=None,
):
    si_barrier_cert = si_barrier_with_connectivity_and_boundary(
        barrier_gain=barrier_gain,
        safety_radius=safety_radius + projection_distance,
        magnitude_limit=magnitude_limit,
        boundary_points=boundary_points,
        connectivity_distance=connectivity_distance,
        group_manager=group_manager,
    )

    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping(
        projection_distance=projection_distance
    )

    uni_to_si_dyn = create_uni_to_si_dynamics(projection_distance=projection_distance)

    def f(dxu, x):
        x_si = uni_to_si_states(x)
        dxi = uni_to_si_dyn(dxu, x)
        dxi = si_barrier_cert(dxi, x_si)
        return si_to_uni_dyn(dxi, x)

    return f


def si_barrier_with_connectivity_and_boundary(
    barrier_gain=100,
    safety_radius=0.17,
    magnitude_limit=0.2,
    boundary_points=np.array([-1.6, 1.6, -1.0, 1.0]),
    connectivity_distance=0.5,
    group_manager=None,
):
    def f(dxi, x):
        lock = threading.Lock()
        # Initialize some variables for computational savings
        N = dxi.shape[1]
        num_constraints = int(comb(N, 2)) + 4 * N
        A = np.zeros((num_constraints, 2 * N))
        b = np.zeros(num_constraints)
        H = 2 * np.identity(2 * N)

        count = 0
        for i in range(N - 1):
            for j in range(i + 1, N):
                error = x[:, i] - x[:, j]
                h = (error[0] * error[0] + error[1] * error[1]) - np.power(
                    safety_radius, 2
                )

                A[count, (2 * i, (2 * i + 1))] = -2 * error
                A[count, (2 * j, (2 * j + 1))] = 2 * error
                b[count] = barrier_gain * np.power(h, 3)
                # print(f"A: {b[count]}")

                count += 1

        for k in range(N):
            # Pos Y
            A[count, (2 * k, 2 * k + 1)] = np.array([0, 1])
            b[count] = (
                0.4
                * barrier_gain
                * (boundary_points[3] - safety_radius / 2 - x[1, k]) ** 3
            )
            count += 1

            # Neg Y
            A[count, (2 * k, 2 * k + 1)] = -np.array([0, 1])
            b[count] = (
                0.4
                * barrier_gain
                * (-boundary_points[2] - safety_radius / 2 + x[1, k]) ** 3
            )
            count += 1

            # Pos X
            A[count, (2 * k, 2 * k + 1)] = np.array([1, 0])
            b[count] = (
                0.4
                * barrier_gain
                * (boundary_points[1] - safety_radius / 2 - x[0, k]) ** 3
            )
            count += 1

            # Neg X
            A[count, (2 * k, 2 * k + 1)] = -np.array([1, 0])
            b[count] = (
                0.4
                * barrier_gain
                * (-boundary_points[0] - safety_radius / 2 + x[0, k]) ** 3
            )
            count += 1
        if (
            connectivity_distance
            and group_manager
            and (group_manager.block_L is not None)
        ):

            L = group_manager.block_L.copy()
            for agent_ndx in range(N):
                if agent_ndx in group_manager.leaders:
                    neighbors = topological_neighbors(L, agent_ndx)
                    for neighbor_ndx in neighbors:
                        error = x[:, agent_ndx] - x[:, neighbor_ndx]
                        h = np.power(connectivity_distance, 2) - (
                            error[0] * error[0] + error[1] * error[1]
                        )

                        i = np.zeros((1, 2 * N))
                        i[0, (2 * agent_ndx, (2 * agent_ndx + 1))] = 2 * error
                        # i[0, (2 * neighbor_ndx, (2 * neighbor_ndx + 1))] = -2 * error
                        # print(i)
                        A = np.vstack((A, i))
                        # print(b.shape)
                        b = np.hstack((b, barrier_gain * np.power(h, 3)))
                        # print(b.shape)
                        # print(f"I: {b[count]}")
                        count += 1
        # Threshold control inputs before QP
        norms = np.linalg.norm(dxi, 2, 0)
        idxs_to_normalize = norms > magnitude_limit
        dxi[:, idxs_to_normalize] *= magnitude_limit / norms[idxs_to_normalize]

        f = -2 * np.reshape(dxi, (2 * N, 1), order="F")
        b = np.reshape(b, (-1, 1), order="F")
        result = qp(matrix(H), matrix(f), matrix(A), matrix(b))["x"]
        # result = solver2.solve_qp(H, f, A, b, 0)[0]

        return np.reshape(result, (2, N), order="F")

    return f
