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

from run.auto_runner import *


def task_mapping(task_name: str) -> type(AutoRunnerBase):
    task_dict = {
        "exploration": AutoRunnerExplore,
        "crossing": AutoRunnerCross,
        "flocking": AutoRunnerFlocking,
        "shaping": AutoRunnerShaping,
        "bridging": AutoRunnerBridging,
        "circling": AutoRunnerCircling,
        "encircling": AutoRunnerEncircling,
        "covering": AutoRunnerCovering,
        "clustering": AutoRunnerClustering,
        "pursuing": AutoRunnerPursuing,
    }
    return task_dict[task_name]


def config_mapping(task_name: str) -> str:
    return task_name + "_config.json"


if __name__ == "__main__":
    task_name = "encircling"
    runner_class = task_mapping(task_name)
    config_file = config_mapping(task_name)
    runner = runner_class(
        env_config_path=f"../config/env/{config_file}",
        workspace_path=task_name,
        # workspace_path='metagpt',
        # workspace_path='cap/cross',
        experiment_duration=15,
        run_mode="rerun",
        # target_pkl='video_critic.pkl',
        # target_pkl='None',
        # script_name='run_meta.py',
        # script_name='run_cap.py',
        max_speed=2.0,
        tolerance=0.15,
    )

    # 人工复核，哪些任务需要重新跑，写在下面
    # exp_list = ['2024-10-11_14-59-04', ]
    exp_list = None

    # exp_list = sorted(extra_exp(f"../workspace/{runner.experiment_path}", out_type='name'))

    runner.run(exp_list=exp_list)
