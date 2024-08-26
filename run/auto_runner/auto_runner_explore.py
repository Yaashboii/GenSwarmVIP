import math
import os
import json
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from modules.deployment.gymnasium_env import GymnasiumExplorationEnvironment
from run.auto_runner import AutoRunnerBase
from run.utils import evaluate_landmark_visits, check_collisions


class AutoRunnerExplore(AutoRunnerBase):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 max_speed=1.0,
                 tolerance=0.05):
        env = GymnasiumExplorationEnvironment(env_config_path)
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
        (all_landmarks_visited,
         landmark_visit_ratio,
         average_visit_step) = evaluate_landmark_visits(run_result)
        (collision,
         collision_frequency,
         collision_severity_sum) = check_collisions(
            run_result, tolerance=self.tolerance)
        return {
            "all_landmarks_visited": all_landmarks_visited,
            "landmark_visit_ratio": landmark_visit_ratio,
            "average_visit_step": average_visit_step,
            "collision": collision,
            "collision_frequency": collision_frequency,
            "collision_severity_sum": collision_severity_sum
        }

    def analyze_all_results(self, experiment_dirs=None):
        pass
