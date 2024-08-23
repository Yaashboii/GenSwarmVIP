import math
import os
import json
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from modules.deployment.gymnasium_env import GymnasiumCrossingEnvironment
from auto_runner_base import AutoRunnerBase


class AutoRunnerCross(AutoRunnerBase):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 max_speed=1.0,
                 tolerance=0.05):
        env = GymnasiumCrossingEnvironment(env_config_path, radius=2.20)
        super().__init__(env_config_path=env_config_path,
                         workspace_path=workspace_path,
                         experiment_duration=experiment_duration,
                         run_mode=run_mode,
                         target_pkl=target_pkl,
                         script_name=script_name,
                         max_speed=max_speed,
                         tolerance=tolerance,
                         env=env)

    def analyze_result(self, run_result):
        def are_trajectories_equal_length(data):
            lengths = [len(info["trajectory"]) for info in data.values()]
            return all(length == lengths[0] for length in lengths)

        def collision_check(data, tolerance=0.1):
            target_entities = {id: info for id, info in data.items() if info["target"] is not None}

            num_timesteps = len(next(iter(data.values()))["trajectory"])
            collision_count = 0
            collision_severity_sum = 0

            processed_pairs = set()

            for id1, info1 in target_entities.items():
                for t in range(num_timesteps):
                    pos1 = info1["trajectory"][t]
                    for id2, info2 in data.items():
                        if id1 == id2 or (id2, id1) in processed_pairs:
                            continue
                        pos2 = info2["trajectory"][t]
                        distance = math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
                        if distance + tolerance < (info1["size"] + info2["size"]):
                            collision_count += 1
                            overlap = info1["size"] + info2["size"] - distance
                            collision_severity_sum += overlap / (info1["size"] + info2["size"])
                            processed_pairs.add((id1, id2))

            num_target_entities = len(target_entities)
            collision_frequency = collision_count / (
                    num_target_entities * num_timesteps) if num_target_entities > 0 else 0
            collision_severity_mean = collision_severity_sum / collision_count if collision_count > 0 else 0

            return collision_count > 0, collision_frequency, collision_severity_mean, collision_severity_sum

        def target_achieve(data, tolerance=0.1):
            total_distance_ratio = 0
            achieved_targets = 0
            num_targets = 0
            total_steps_ratio = 0

            for entity_id, info in data.items():
                target = info["target"]
                if target is None:
                    continue
                num_targets += 1
                initial_position = info["trajectory"][0]
                initial_distance = math.sqrt(
                    (target[0] - initial_position[0]) ** 2 + (target[1] - initial_position[1]) ** 2)

                steps_to_target = len(info["trajectory"])
                final_distance = initial_distance
                for t, position in enumerate(info["trajectory"]):
                    current_distance = math.sqrt(
                        (target[0] - position[0]) ** 2 + (target[1] - position[1]) ** 2)
                    if current_distance <= tolerance:
                        steps_to_target = t + 1
                        final_distance = current_distance
                        break
                if final_distance <= tolerance:
                    achieved_targets += 1
                    total_steps_ratio += steps_to_target / len(info["trajectory"])
                distance_ratio = final_distance / initial_distance if initial_distance > 0 else 1
                total_distance_ratio += distance_ratio

                print(
                    f"Entity {entity_id}: Initial Distance: {initial_distance}, Final Distance: {final_distance}, Ratio: {distance_ratio}, Steps to Target: {steps_to_target}")

            target_achieve_ratio = achieved_targets / num_targets if num_targets > 0 else 0
            average_distance_ratio = total_distance_ratio / num_targets if num_targets > 0 else 0
            average_steps_ratio = total_steps_ratio / achieved_targets if achieved_targets > 0 else 'inf'

            return achieved_targets == num_targets, target_achieve_ratio, average_distance_ratio, average_steps_ratio

        if not are_trajectories_equal_length(run_result):
            raise ValueError("Trajectories have different lengths")

        collision, collision_frequency, collision_severity_mean, collision_severity_sum = collision_check(
            run_result, tolerance=self.tolerance)
        target_achieve, target_achieve_ratio, average_distance_ratio, average_steps_ratio = target_achieve(
            run_result, tolerance=2 * self.tolerance)

        return {
            'collision_occurred': collision,
            'collision_frequency': collision_frequency,
            'collision_severity_average': collision_severity_mean,
            'collision_severity_sum': collision_severity_sum,
            'target_achieved': target_achieve,
            'target_achievement_ratio': target_achieve_ratio,
            'distance_ratio_average': average_distance_ratio,
            'steps_ratio_average': average_steps_ratio
        }

    def analyze_all_results(self, experiment_dirs=None):
        metric_name = ["collision_frequency", "collision_severity_sum", "distance_ratio_average",
                       "target_achievement_ratio",
                       "steps_ratio_average"]
        if not experiment_dirs:
            experiment_dirs = sorted(self.get_experiment_directories())
        no_collision_files = []
        error_files = []
        no_target_achieve_files = []
        no_collision_and_target_achieve = []
        code_lines = []

        exp_data = {}
        for experiment in experiment_dirs:
            result_path = os.path.join(f"../workspace/{self.experiment_path}", experiment, 'result.json')
            if os.path.exists(result_path):
                with open(result_path, 'r') as f:
                    result_data = json.load(f)
                    analysis = result_data.get('analysis', {})
                    if not analysis.get('collision_occurred'):
                        no_collision_files.append(experiment)
                    if not analysis.get('target_achieved'):
                        no_target_achieve_files.append(experiment)
                    if not analysis.get('collision_occurred') and analysis.get('target_achieved'):
                        no_collision_and_target_achieve.append(experiment)
                    run_result = result_data.get('run_code_result', {})
                    if run_result.get('error') == True:
                        error_files.append(experiment)
                        print(run_result.get('reason'))

                    exp_data[experiment] = {}
                    for m in metric_name:
                        exp_data[experiment][m] = analysis.get(m)
                        if m == 'steps_ratio_average' and analysis.get(m) == 'inf':
                            print(experiment, "inf steps_ratio_average")
                            exp_data[experiment][m] = 0
            code_path = os.path.join(f"../workspace/{self.experiment_path}", experiment, 'functions.py')
            if os.path.exists(code_path):
                code_lines.append(self.analyze_code(code_path))
        mean_metric_value = {}
        for m in metric_name:
            mean_metric_value[m] = np.mean([exp_data[i][m] for i in exp_data.keys()])

        metrics_data = [
            (
                [mean_metric_value["collision_frequency"], mean_metric_value["collision_severity_sum"]],
                ['Collision Frequency', 'Collision Severity'],
                'Value',
                'Collision Metrics',
                ['purple', 'orange'],
                'collision_metrics.png'
            ),
            (
                [mean_metric_value["distance_ratio_average"], mean_metric_value["target_achievement_ratio"],
                 mean_metric_value["steps_ratio_average"]],
                ['Distance Ratio', 'Target Achieve Ratio', 'Steps Ratio'],
                'Ratio',
                'Distance Metrics',
                ['cyan', 'blue', 'green'],
                'distance_metrics.png'
            ),
        ]

        for data, labels, ylabel, title, colors, filename in metrics_data:
            self.plot_and_print_results(data, labels, ylabel, title, colors, filename)

        bug_rate = len(error_files) / len(experiment_dirs) if experiment_dirs else 0

        no_collision_rate = len(no_collision_files) / len(
            experiment_dirs) if experiment_dirs else 0
        target_achieve_rate = 1 - len(no_target_achieve_files) / len(experiment_dirs) if experiment_dirs else 0
        no_collision_and_target_rate = len(no_collision_and_target_achieve) / len(
            experiment_dirs) if experiment_dirs else 0
        code_lines_avg = np.mean(code_lines) / 200
        self.plot_and_print_results(
            [bug_rate, no_collision_rate, target_achieve_rate, no_collision_and_target_rate, code_lines_avg],
            ['Bug', 'No Collision', 'Target Achieve', 'No Collision & Target Achieve', 'code_lines'],
            'Rate',
            'Experiment Outcomes',
            ['red', 'yellow', 'blue', 'green', 'orange'],
            'experiment_outcomes.png'
        )
        colors = ['#96B6C5', '#ADC4CE', '#EEE0C9', '#F1F0E8']

        start_color = "#0000FF"  # 蓝色
        end_color = "#FF0000"  # 红色
        cmap = LinearSegmentedColormap.from_list("my_colormap", [start_color, end_color])
        for m in metric_name:
            num_colors = len(exp_data.keys())
            colors_list = [cmap(i / (num_colors - 1)) for i in range(num_colors)]
            self.plot_and_print_results(
                data=[exp_data[i][m] for i in exp_data.keys()],
                labels=exp_data.keys(),
                ylabel=m,
                title=f"Experiment Outcomes in {m}",
                colors=colors_list,
                save_filename=f'{m} metric.png',
                rotation=True,
            )


if __name__ == "__main__":
    runner = AutoRunnerCross("../config/env_config.json",
                             workspace_path='ablation/human_feedback',
                             # workspace_path='metagpt',
                             # workspace_path='cap/cross',
                             experiment_duration=30,
                             run_mode='analyze',
                             # target_pkl='video_critic.pkl',
                             target_pkl='None',
                             # script_name='run_meta.py',
                             script_name='run_cap.py',
                             max_speed=0.75,
                             tolerance=0.15
                             )
    # 人工复核，哪些任务需要重新跑，写在下面
    # experiment_list = ['2024-07-21_18-26-12', ]
    experiment_list = None
