import numpy as np


def is_near(pos, target, margin):
    """
    Determine whether the object at pos is near the target (within some margin)
    :param pos: [x, y, z]
    :param target: [x, y, z]
    :param margin: allowed distance
    :return: True if obj is near target, False otherwise
    """
    dist = np.array(pos) - np.array(target)
    dist[1] = 0
    # if the distance (without y) is smaller than some margin
    if np.linalg.norm(dist) <= margin:
        return True
    else:
        return False


def cos_between(dir1, dir2):
    """
    Calculate cosine between two vectors
    :param dir1: np.array([x, y, z])
    :param dir2: np.array([x, y, z])
    :return: cosine between dir1 and dir2
    """
    return np.dot(dir1, dir2) / (np.linalg.norm(dir1) * np.linalg.norm(dir2))


def angle_between(dir1, dir2):
    """
    Calculate angle between two vectors
    :param dir1: np.array([x, y, z])
    :param dir2: np.array([x, y, z])
    :return: angle between dir1 and dir2
    """
    return np.arccos(cos_between(dir1, dir2))


def faces(pos, direction, target, margin):
    """
    Determine whether the object as pos looking in a direction faces a target (with some error margin)
    :param pos: [x, y, z]
    :param direction: [x, y, z]
    :param target: [x, y, z]
    :param margin: allowed deviation in degrees
    :return: True if obj faces target, False otherwise
    """
    # return true if person at pos looking in dir faces target (with some error)
    target_dir = np.array(target) - np.array(pos)
    target_dir[1] = 0
    direction[1] = 0
    if np.rad2deg(np.arccos(cos_between(np.array(direction), target_dir))) <= margin:
        return True
    else:
        return False
