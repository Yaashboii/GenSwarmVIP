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
import argparse


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


def main():
    # 创建解析器
    parser = argparse.ArgumentParser(description="Run the robot experiment.")
    # 添加 exp_batch 参数
    parser.add_argument(
        "--exp_batch",
        type=int,
        default=1,
        help="The number of experiment batches to run",
    )

    # 解析参数
    args = parser.parse_args()

    task_name = "pursuing"
    runner_class = task_mapping(task_name)
    config_file = config_mapping(task_name)
    runner = runner_class(
        env_config_path=f"../config/env/{config_file}",
        workspace_path=task_name,
        experiment_duration=8,
        exp_batch=args.exp_batch,  # 使用传入的 exp_batch 参数
        run_mode="analyze",
        max_speed=4.5,
        tolerance=0.15,
    )

    # 人工复核，哪些任务需要重新跑，写在下面
    # exp_list = ['2024-10-28_01-49-03', '2024-10-28_01-49-05', '2024-10-28_01-49-09', '2024-10-28_01-49-15',
    #             '2024-10-28_01-49-19', '2024-10-28_01-49-27', '2024-10-28_01-51-49']
    exp_list = None
    # exp_list = ['2024-10-31_17-05-42']

    runner.run(exp_list=exp_list)


if __name__ == "__main__":
    main()
