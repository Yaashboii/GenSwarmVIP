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

import operator

from modules.deployment.gymnasium_env import GymnasiumCirclingEnvironment
from run.auto_runner import AutoRunnerBase
from run.utils import evaluate_robot_circle_similarity


class AutoRunnerCircling(AutoRunnerBase):
    def __init__(
        self,
        env_config_path,
        workspace_path,
        experiment_duration,
        run_mode="rerun",
        target_pkl="WriteRun.pkl",
        script_name="run.py",
        max_speed=1.0,
        tolerance=0.05,
    ):
        env = GymnasiumCirclingEnvironment(env_config_path)
        super().__init__(
            env_config_path=env_config_path,
            workspace_path=workspace_path,
            experiment_duration=experiment_duration,
            run_mode=run_mode,
            target_pkl=target_pkl,
            script_name=script_name,
            max_speed=max_speed,
            tolerance=tolerance,
            env=env,
        )

    def analyze_result(self, run_result) -> dict[str, float]:
        line_similarity = evaluate_robot_circle_similarity(
            run_result, expected_circle_center=(0, 0), expected_circle_radius=1
        )
        return line_similarity

    def setup_success_conditions(self) -> list[tuple[str, operator, float]]:
        return [
            ("center_distance", operator.lt, 0.1),
            ("radius_difference", operator.lt, 0.1),
        ]
