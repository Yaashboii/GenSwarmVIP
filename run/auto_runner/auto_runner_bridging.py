import operator

import numpy as np

from modules.deployment.gymnasium_env import GymnasiumBridgingEnvironment
from run.auto_runner import AutoRunnerBase
from run.utils import evaluate_shape_similarity


class AutoRunnerBridging(AutoRunnerBase):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 max_speed=1.0,
                 tolerance=0.05):
        env = GymnasiumBridgingEnvironment(env_config_path)
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
        robots_num = self.env.num_robots
        target_y = np.linspace(-2, 2, robots_num)
        target_x = np.zeros(robots_num)
        target_line = np.array([target_x, target_y]).T
        line_similarity = evaluate_shape_similarity(run_result, target_shape=target_line)
        return line_similarity

    def setup_success_conditions(self) -> list[tuple[str, operator, float]]:
        return [
            ("procrustes_distance", operator.lt, 0.2),
        ]
