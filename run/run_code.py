from run.auto_runner import *


def task_mapping(task_name: str) -> type(AutoRunnerBase):
    task_dict = {
        "exploration": AutoRunnerExplore,
        "crossing": AutoRunnerCross,
    }
    return task_dict[task_name]


if __name__ == "__main__":
    task_name = 'exploration'
    runner_class = task_mapping(task_name)
    runner = runner_class(env_config_path="../config/env_config.json",
                          workspace_path='exploration',
                          # workspace_path='metagpt',
                          # workspace_path='cap/cross',
                          experiment_duration=30,
                          run_mode='rerun',
                          # target_pkl='video_critic.pkl',
                          target_pkl='None',
                          # script_name='run_meta.py',
                          # script_name='run_cap.py',
                          max_speed=0.75,
                          tolerance=0.15)

    # 人工复核，哪些任务需要重新跑，写在下面
    # experiment_list = ['2024-07-21_18-26-12', ]
    # exp_list = sorted(extra_exp(f"../workspace/{runner.experiment_path}", out_type='name'))

    runner.run(exp_list=None)
