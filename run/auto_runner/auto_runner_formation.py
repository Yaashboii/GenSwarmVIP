from modules.deployment.gymnasium_env import GymnasiumTransportationEnvironment
from modules.deployment.gymnasium_env.gymnasium_formation_env import GymnasiumFormationEnvironment
from run.auto_runner import AutoRunnerBase


class AutoRunnerFormation(AutoRunnerBase):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 max_speed=1.0,
                 tolerance=0.05):
        env = GymnasiumFormationEnvironment(env_config_path)
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
        pass

    def analyze_all_results(self, experiment_dirs=None):
        pass
