"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""

import numpy as np

robot_info = {
    "id": None,
    "position": np.array([0.0, 0.0]),
    "radius": 0.0,
    "velocity": np.array([0.0, 0.0]),
}

ros_initialized = False
init_position = None
target_position = None
obstacles_info = []
other_robots_info = []
prey_positions = []
moveable_objects = []
formation_points = [(1, -1), (1, 1), (0, 0), (1, 0), (2, 0), (2, 2)]




def initialize_ros_node(robot_id):
    pass

def publish_velocities(event):
    pass


def get_all_robots_info():
    global other_robots_info
    return None


def get_self_position():
    return robot_info["position"]


def get_self_velocity():
    return robot_info["velocity"]


def get_self_radius():
    return robot_info["radius"]


def set_self_velocity(velocity):
    robot_info["velocity"] = np.array(velocity)


def get_surrounding_robots_info():
    return other_robots_info


def get_surrounding_obstacles_info():
    return obstacles_info


def get_prey_position():
    return prey_positions[0] if prey_positions else None


def get_self_id():
    return robot_info["id"]


def get_target_position():
    return target_position


def get_target_formation_points():
    return formation_points
