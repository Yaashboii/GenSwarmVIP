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

import time
import traceback
from traceback import TracebackException


import numpy as np

from modules.deployment.utils.char_points_generate import validate_contour_points


class Manager:
    def __init__(self, env, max_speed=1.2, real=False):
        self.last_time = 0
        self.env = env
        self._pub_list = []
        self._robots = env.get_entities_by_type("Robot") + env.get_entities_by_type(
            "Leader"
        )
        self._max_speed = max_speed
        robot_start_index = min(self._robots, key=lambda x: x.id).id
        robot_end_index = max(self._robots, key=lambda x: x.id).id

        self.robotID_velocity = {
            robot.id: np.array([0, 0], dtype=float) for robot in self._robots
        }

    def publish_observations(self, obs=None):
        if obs:
            observation = obs
        else:
            observation = self.env.get_observation()


    def get_target_positions_callback(self, request):
        pass

    def get_char_points_callback(self, request):
        char = request.character
        try:
            sampled_points = validate_contour_points(char)
        except Exception as e:
            print(f"Error occurred: {e}")
            traceback.print_exc()

        return

    def clear_velocity(self):
        self.robotID_velocity = {
            robot.id: np.array([0, 0], dtype=float) for robot in self._robots
        }
