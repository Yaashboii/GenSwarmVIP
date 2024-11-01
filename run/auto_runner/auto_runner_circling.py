import operator

from modules.deployment.gymnasium_env import GymnasiumCirclingEnvironment
from run.auto_runner import AutoRunnerBase
from run.utils import evaluate_robot_circle_similarity


class AutoRunnerCircling(AutoRunnerBase):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 exp_batch=1,
                 max_speed=1.0,
                 tolerance=0.05):
        env = GymnasiumCirclingEnvironment(env_config_path)
        super().__init__(env_config_path=env_config_path,
                         workspace_path=workspace_path,
                         experiment_duration=experiment_duration,
                         run_mode=run_mode,
                         target_pkl=target_pkl,
                         script_name=script_name,
                         max_speed=max_speed,
                         exp_batch=exp_batch,
                         tolerance=tolerance,
                         env=env)

    def analyze_result(self, run_result) -> dict[str, float]:
        line_similarity = evaluate_robot_circle_similarity(run_result, expected_circle_center=(0, 0),
                                                           expected_circle_radius=1)
        return line_similarity

    def setup_success_conditions(self) -> list[tuple[str, operator, float]]:
        return [
            ("center_distance", operator.lt, 0.1),
            ("radius_difference", operator.lt, 0.1)
        ]
