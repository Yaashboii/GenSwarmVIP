import math
import os
import json
from abc import ABC

import numpy as np
from matplotlib.colors import LinearSegmentedColormap
import operator
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
                 exp_batch=1,
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
                         exp_batch=exp_batch,
                         env=env)

    def setup_success_conditions(self) -> list[tuple[str, operator, float]]:
        return [
            ("landmark_visit_ratio", operator.ge, 0.99),
        ]

    def analyze_result(self, run_result) -> dict[str, float]:
        landmark_visit = evaluate_landmark_visits(run_result)
        collisions = check_collisions(run_result, tolerance=self.tolerance)
        return landmark_visit | collisions
