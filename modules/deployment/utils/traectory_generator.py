import numpy as np


def generate_arc_trajectory(num_points: int, start_angle: float, end_angle: float, radius: float,
                            center: tuple) -> dict:
    """
    生成一个弧形轨迹，key为时间步，value为坐标点 (x, y)。

    :param num_points: 轨迹点的数量（即时间步的数量）。
    :param start_angle: 起始角度（弧度）。
    :param end_angle: 结束角度（弧度）。
    :param radius: 圆弧的半径。
    :param center: 圆心坐标 (x, y)。
    :return: 包含时间步作为key，轨迹点坐标 (x, y) 作为value的字典。
    """
    time_to_position = {}
    angles = np.linspace(start_angle, end_angle, num_points)

    for t, angle in enumerate(angles):
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        time_to_position[t] = (x, y)

    return time_to_position

