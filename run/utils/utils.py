import numpy as np


def calculate_overlap_ratio(pos1, pos2, size1, size2, tolerance) -> float | None:
    """
    Calculate the overlap ratio between two entities if they are colliding.
    :param pos1: Position of the first entity
    :param pos2: Position of the second entity
    :param size1: Size of the first entity
    :param size2: Size of the second entity
    :param tolerance: Allowed tolerance for collision
    :return: overlap ratio if colliding, otherwise None
    """
    distance = np.linalg.norm(pos1 - pos2)
    combined_size = size1 + size2
    if distance + tolerance < combined_size:
        overlap = combined_size - distance
        return overlap / combined_size
    return None
