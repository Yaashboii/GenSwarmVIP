from dynamic_reconfigure.encoding import encode_config

from run.auto_runner import *


def task_mapping(task_name: str) -> type(AutoRunnerBase):
    task_dict = {
        "exploration": AutoRunnerExplore,
        "crossing": AutoRunnerCross,
        'flocking': AutoRunnerFlocking,
        'shaping': AutoRunnerShaping,
        'bridging': AutoRunnerBridging,
        'circling': AutoRunnerCircling,
        'encircling': AutoRunnerEncircling,
        'formation': AutoRunnerFormation,
        'herding': AutoRunnerHerding,
        'covering': AutoRunnerCovering,
    }
    return task_dict[task_name]


def config_mapping(task_name: str) -> str:
    return task_name + '_config.json'


if __name__ == "__main__":
    task_name = 'exploration'
    runner_class = task_mapping(task_name)
    config_file = config_mapping(task_name)
    runner = runner_class(env_config_path=f"../config/env/{config_file}",
                          workspace_path=task_name,
                          # workspace_path='metagpt',
                          # workspace_path='cap/cross',
                          experiment_duration=40,
                          run_mode='rerun',
                          # target_pkl='video_critic.pkl',
                          target_pkl='None',
                          # script_name='run_meta.py',
                          # script_name='run_cap.py',
                          max_speed=0.75,
                          tolerance=0.15)

    # 人工复核，哪些任务需要重新跑，写在下面
    # exp_list = ['2024-08-28_09-31-30', ]
    exp_list = None
    # exp_list = sorted(extra_exp(f"../workspace/{runner.experiment_path}", out_type='name'))

    runner.run(exp_list=exp_list)
