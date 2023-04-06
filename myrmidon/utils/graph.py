import numpy as np


def complete_GL_leaderless(num_bots):
    if num_bots == 1:
        return np.zeros((1, 1)), np.zeros((1, 1))
    elif num_bots == 2:
        followers = np.array([[1, -1], [-1, 1]])
        follower_dists = -followers
        return followers, follower_dists
    followers = num_bots * np.identity(num_bots) - np.ones((num_bots, num_bots))
    # Dists:
    # where k is the distance apart
    # sin(k*np.pi/num_bots)/sin(np.pi/num_bots)
    follower_dists = np.zeros((num_bots, num_bots))
    for k in range(num_bots):
        offset = k
        ones = np.ones(num_bots - (offset))
        dist = np.sin(k * np.pi / num_bots) / np.sin(np.pi / num_bots) * ones
        follower_dists += np.diag(dist, offset) + np.diag(dist, -offset)
    return followers, follower_dists


def complete_GL(num_bots=1):
    def build_graph(num_bots):
        if num_bots == 1:
            return np.zeros((1, 1)), np.zeros((1, 1))
        followers, follower_dists = complete_GL_leaderless(num_bots - 1)
        L = np.zeros((num_bots, num_bots))
        L[1:num_bots, 1:num_bots] = followers
        L[1, 1] += 1
        L[1, 0] = -1
        dists = np.zeros((num_bots, num_bots))
        dists[1:num_bots, 1:num_bots] = follower_dists
        dists[1, 0] = 1
        if num_bots > 2:
            L[2, 2] += 1
            L[2, 0] = -1
            dists[2, 0] = 1
        return -L, dists

    return build_graph


def line_GL(num_bots=1):
    def build_graph(num_bots):
        ones = np.ones(num_bots - 1)
        L = 2 * np.identity(num_bots) - np.diag(ones, 1) - np.diag(ones, -1)
        L[0, 0] = 1
        L[num_bots - 1, num_bots - 1] = 1
        dists = np.diag(ones, 1) + np.diag(ones, -1)
        return -L, dists

    return build_graph


def cycle_GL(num_bots=1):
    def build_graph(num_bots):

        ones = np.ones(num_bots - 1)
        L = 2 * np.identity(num_bots) - np.diag(ones, 1) - np.diag(ones, -1)
        L[num_bots - 1, 0] = -1
        L[0, num_bots - 1] = -1
        dists = np.diag(ones, 1) + np.diag(ones, -1)
        dists[num_bots - 1, 0] = 1
        dists[0, num_bots - 1] = 1
        return -L, dists

    return build_graph


def directed_cycle_GL(num_bots=1):
    def build_graph(num_bots):
        ones = np.ones(num_bots - 1)
        L = np.identity(num_bots) - np.diag(ones, 1)
        L[num_bots - 1, 0] = -1
        dists = np.diag(ones, 1)
        dists[num_bots - 1, 0] = 1
        return -L, dists

    return build_graph


def _rigid_minimal_GL_leaderless(num_bots):
    if num_bots == 1:
        return np.zeros((1, 1)), np.zeros((1, 1))
    elif num_bots == 2:
        followers = np.array([[1, -1], [-1, 1]])
        follower_dists = -followers
        return followers, follower_dists
    rigid_connects = np.resize([1, 0], (num_bots - 2,))
    # * (1 / np.cos(np.pi /  num_bots))
    adjustment = 2 * np.cos(np.pi / num_bots)
    rigid_adjusted = (
        -(np.diag(rigid_connects, 2) + np.diag(rigid_connects, -2)) * adjustment
    )
    ones = np.ones(num_bots - 1)
    followers = (
        -np.diag(ones, 1)
        - np.diag(ones, -1)
        - np.diag(rigid_connects, 2)
        - np.diag(rigid_connects, -2)
    )
    followers[num_bots - 1, 0] = -1
    followers[0, num_bots - 1] = -1
    followers[num_bots - 2, 0] = -1
    followers[0, num_bots - 2] = -1
    followers -= np.diag(np.sum(followers, 0))
    follower_dists = (np.diag(ones, 1) + np.diag(ones, -1)) - rigid_adjusted
    follower_dists[num_bots - 1, 0] = 1
    follower_dists[0, num_bots - 1] = 1
    follower_dists[num_bots - 2, 0] = 1 * adjustment
    follower_dists[0, num_bots - 2] = 1 * adjustment
    return followers, follower_dists


def rigid_minimal_GL(num_bots=1):
    def build_graph(num_bots):
        if num_bots == 1:
            return np.zeros((1, 1)), np.zeros((1, 1))
        followers, follower_dists = _rigid_minimal_GL_leaderless(num_bots - 1)
        L = np.zeros((num_bots, num_bots))
        L[1:num_bots, 1:num_bots] = followers
        L[1, 1] += 1
        L[1, 0] = -1
        L[0, 1] = -1
        dists = np.zeros((num_bots, num_bots))
        dists[1:num_bots, 1:num_bots] = follower_dists
        dists[1, 0] = 1
        dists[0, 1] = 1
        if num_bots > 2:
            L[2, 2] += 1
            L[2, 0] = -1
            L[0, 2] = -1
            dists[2, 0] = 1
            dists[0, 2] = 1
        return -L, dists

    return build_graph


def topological_neighbors(L, agent):
    copied = L.copy()
    row = copied[agent, :]
    row[agent] = 0
    return np.where(row != 0)[0]
