import math
import operator
import os
import json
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from modules.deployment.gymnasium_env import GymnasiumFlockingEnvironment, GymnasiumClusteringEnvironment
from run.auto_runner import AutoRunnerBase
from run.utils import evaluate_trajectory_similarity, evaluate_trajectory_pattern, evaluate_robot_quadrant_positions


class AutoRunnerClustering(AutoRunnerBase):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 max_speed=1.0,
                 tolerance=0.05):
        env = GymnasiumClusteringEnvironment(env_config_path)
        super().__init__(env_config_path=env_config_path,
                         workspace_path=workspace_path,
                         experiment_duration=experiment_duration,
                         run_mode=run_mode,
                         target_pkl=target_pkl,
                         script_name=script_name,
                         max_speed=max_speed,
                         tolerance=tolerance,
                         env=env)

    def analyze_result(self, run_result) -> dict[str, float]:
        target_regions = {
            1: (0.5, 2, 0.5, 2),
            2: (-2, -0.5, 0.5, 2),
            3: (-2, -0.5, -2, -0.5),
            4: (0.5, 2, -2, -0.5)
        }
        similarity = evaluate_robot_quadrant_positions(run_result, target_regions=target_regions)
        return similarity

    def setup_success_conditions(self) -> list[tuple[str, operator, float]]:
        return [
            ("achievement_ratio", operator.ge, 0.99),
        ]
