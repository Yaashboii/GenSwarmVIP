import operator

from modules.deployment.gymnasium_env import GymnasiumTransportationEnvironment, GymnasiumHerdingEnvironment
from modules.deployment.gymnasium_env.gymnasium_covering_env import GymnasiumCoveringEnvironment
from run.auto_runner import AutoRunnerBase
from run.utils import evaluate_robot_final_positions


class AutoRunnerCovering(AutoRunnerBase):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 exp_batch=1,
                 max_speed=1.0,
                 tolerance=0.05):
        env = GymnasiumCoveringEnvironment(env_config_path)
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

    def analyze_result(self, run_result) -> dict[str, float]:
        even_metric = evaluate_robot_final_positions(run_result)
        return even_metric

    def setup_success_conditions(self) -> list[tuple[str, operator, float]]:
        return [
            ("area_ratio", operator.ge, 0.80),
            ("variance_nearest_neighbor_distance", operator.le, 0.1)
        ]
