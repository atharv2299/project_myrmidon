import math
import time

import numpy as np


def chunks(seq, num_chunks):
    chunk_size = int(math.ceil(len(seq) / num_chunks))
    # TODO: Look into floor and then modulo to add on the remainder
    # TODO: Replace with numpy parts(?) or whatever it is
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
