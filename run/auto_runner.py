import math
import os
import queue
import shutil
import threading
import subprocess
import time

import imageio
import rospy
import json
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from tqdm import tqdm

from experiment.ablation.utils import extra_exp
from modules.deployment.gymnasium_env import GymnasiumCrossEnvironment
from modules.deployment.utils.manager import Manager


class AutoRunner:
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 max_speed=1.0,
                 tolerance=0.05):
        self.env_config_path = env_config_path
        self.experiment_path = workspace_path
        self.experiment_duration = experiment_duration
        self.env = GymnasiumCrossEnvironment(self.env_config_path, radius=2.20)
        self.env.reset()
        self.results = {}
        self.target_pkl = target_pkl
        self.manager = Manager(self.env, max_speed=max_speed)
        self.script_name = script_name
        self.stop_event = threading.Event()
        self.run_mode = run_mode
        self.tolerance = tolerance
        self.frames = []

    def get_experiment_directories(self):
        directories = []
        for item in os.listdir(f"../workspace/{self.experiment_path}"):
            item_path = os.path.join(f"../workspace/{self.experiment_path}", item)
            if os.path.isdir(item_path):
                if self.run_mode == 'continue':
                    if not self.experiment_completed(item_path):
                        directories.append(item)
                elif self.run_mode == 'rerun':
                    directories.append(item)
                elif self.run_mode == 'analyze':
                    if self.experiment_completed(item_path):
                        directories.append(item)
        return directories

    def experiment_completed(self, path):
        result_file = os.path.join(path, 'result.json')
        return os.path.exists(result_file)

    def save_experiment_result(self, path, result, analysis, run_code_result):
        def convert_to_serializable(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, dict):
                return {k: convert_to_serializable(v) for k, v in obj.items()}
            if isinstance(obj, list):
                return [convert_to_serializable(i) for i in obj]
            return obj

        serializable_result = convert_to_serializable(result)
        serializable_analysis = convert_to_serializable(analysis)
        combined_result = {
            'run_code_result': run_code_result,
            'analysis': serializable_analysis,
            'experiment_data': serializable_result,
        }
        result_file = os.path.join(path, 'result.json')
        os.makedirs(path, exist_ok=True)
        with open(result_file, 'w') as f:
            json.dump(combined_result, f, indent=4)

    def save_frame_to_disk(self, frame, frame_index):
        frame_path = os.path.join(self.frame_dir, f'frame_{frame_index:06d}.png')
        imageio.imwrite(frame_path, frame)

    def save_frames_as_animations(self, experiment_id):
        # Save as GIF
        gif_path = os.path.join(f"../workspace/{self.experiment_path}", experiment_id, 'animation.gif')
        imageio.mimsave(gif_path, self.frames, fps=10)
        print(f"Saved animation for experiment {experiment_id} as GIF at {gif_path}")

        # Save as MP4
        mp4_path = os.path.join(f"../workspace/{self.experiment_path}", experiment_id, 'animation.mp4')
        height, width, layers = self.frames[0].shape
        size = (width, height)
        out = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*'mp4v'), self.env.FPS, size)

        for frame in self.frames:
            out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

        out.release()
        print(f"Saved animation for experiment {experiment_id} as MP4 at {mp4_path}")

        self.frames.clear()

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

    def run_single_experiment(self, experiment_id, result_queue):
        obs, infos = self.env.reset()

        def init_result(infos):
            result = {}
            for entity_id in infos:
                result[entity_id] = {"size": 0, "target": None, "trajectory": []}
                result[entity_id]["size"] = infos[entity_id]["size"]
                result[entity_id]["target"] = infos[entity_id]["target_position"]
                result[entity_id]["trajectory"].append(infos[entity_id]["position"])
            return result

        result = init_result(infos)
        self.manager.publish_observations(infos)
        rate = rospy.Rate(self.env.FPS)
        start_time = rospy.get_time()
        experiment_path = os.path.join(f"../workspace/{self.experiment_path}", experiment_id)

        self.frame_dir = os.path.join(experiment_path, 'frames')
        os.makedirs(self.frame_dir, exist_ok=True)

        frame_index = 0
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            current_time = rospy.get_time()
            if current_time - start_time > self.experiment_duration:
                break
            action = self.manager.robotID_velocity
            self.manager.clear_velocity()
            obs, reward, termination, truncation, infos = self.env.step(action=action)
            for entity_id in infos:
                result[entity_id]["trajectory"].append(infos[entity_id]["position"])
            self.frames.append(self.env.render())
            self.manager.publish_observations(infos)
            rate.sleep()
        print(f"Experiment {experiment_id} completed successfully.")
        self.save_frames_as_animations(experiment_id)

        result_queue.put({'source': 'run_single_experiment', 'result': result})

    def run_code(self, experiment, script_name, result_queue):
        experiment_path = os.path.join(self.experiment_path, experiment)
        command = ['python', '../modules/framework/actions/run_code.py', '--data', experiment_path, '--timeout',
                   str(self.experiment_duration - 3), '--target_pkl', self.target_pkl,
                   '--script', f"{script_name}"]

        try:
            result = subprocess.run(command, timeout=self.experiment_duration - 2, capture_output=True, text=True,
                                    check=True)
            result_queue.put({'source': 'run_code', 'error': False, 'reason': ''})
        except subprocess.TimeoutExpired:
            print(f"\nExperiment {experiment} timed out and was terminated.")
            result_queue.put({'source': 'run_code', 'error': False, 'reason': 'timeout'})

        except subprocess.CalledProcessError as e:
            print(f"An error occurred while running {experiment}: {e}")
            print(f"Errors: {e.stderr}")
            result_queue.put({'source': 'run_code', 'error': True, 'reason': e.stderr})

    def setup_metagpt(self, directory):
        # 确保目录存在
        if not os.path.exists(directory):
            print(f"目录 {directory} 不存在")
            return
        # 如果目录下没有py文件，遍历所以的文件夹找到有py文件的，将其内部的所以py文件复制到directory下
        has_py_files = any(file.endswith('.py') for file in os.listdir(directory))
        if not has_py_files:
            for root, _, files in os.walk(directory):
                for file in files:
                    if file.endswith('.py'):
                        src_file = os.path.join(root, file)
                        dest_file = os.path.join(directory, file)
                        shutil.copy(src_file, dest_file)
                        print(f"复制文件 {src_file} 到 {dest_file}")
        # 将apis.py文件复制到directory下
        source_file = os.path.join('../modules/deployment/execution_scripts', 'apis_meta.py')
        if os.path.exists(source_file):
            shutil.copy(source_file, directory)
        else:
            print(f"文件 {source_file} 不存在")
            return
        source_file = os.path.join('../modules/deployment/execution_scripts', 'run_meta.py')
        if os.path.exists(source_file):
            shutil.copy(source_file, directory)
        else:
            print(f"文件 {source_file} 不存在")
            return

        # 遍历文件夹下所有python文件，开头加上from apis import *
        for root, _, files in os.walk(directory):
            for file in files:
                if file.endswith('.py') and file != 'apis_meta.py':
                    file_path = os.path.join(root, file)
                    with open(file_path, 'r+', encoding='utf-8') as f:
                        content = f.read()
                        if not content.startswith('from apis_meta import *'):
                            f.seek(0, 0)
                            f.write('from apis_meta import *\n' + content)

        # 查找是否存在time.sleep(xx),或者rate.sleep() 如果有将其注释
        import re
        sleep_patterns = [
            re.compile(r'(\s*)time\.sleep\(\s*.*?\s*\)'),
            re.compile(r'(\s*)rate\.sleep\(\s*.*?\s*\)')
        ]

        for root, _, files in os.walk(directory):
            for file in files:
                if file.endswith('.py'):
                    file_path = os.path.join(root, file)
                    with open(file_path, 'r', encoding='utf-8') as f:
                        lines = f.readlines()

                    with open(file_path, 'w', encoding='utf-8') as f:
                        for line in lines:
                            for pattern in sleep_patterns:
                                match = pattern.match(line)
                                if match:
                                    line = f"{match.group(1)}# {line}"
                                    break
                            f.write(line)

    def setup_cap(self, directory):
        source_file = os.path.join('../modules/deployment/execution_scripts', 'apis_meta.py')

        if os.path.exists(source_file):
            shutil.copy(source_file, directory)
        else:
            print(f"文件 {source_file} 不存在")
            return
        source_file = os.path.join('../modules/deployment/execution_scripts', 'run_cap.py')
        if os.path.exists(source_file):
            shutil.copy(source_file, directory)
        else:
            print(f"文件 {source_file} 不存在")
            return

        # 遍历文件夹下所有python文件，开头加上from apis import *
        for root, _, files in os.walk(directory):
            for file in files:
                if file.endswith('.py') and file != 'apis_meta.py':
                    file_path = os.path.join(root, file)
                    with open(file_path, 'r+', encoding='utf-8') as f:
                        content = f.read()
                        if not content.startswith('from apis_meta import *'):
                            f.seek(0, 0)
                            f.write('from apis_meta import *\n' + content)
        import re
        sleep_patterns = [
            re.compile(r'(\s*)time\.sleep\(\s*.*?\s*\)'),
            re.compile(r'(\s*)rate\.sleep\(\s*.*?\s*\)')
        ]

        for root, _, files in os.walk(directory):
            for file in files:
                if file.endswith('.py'):
                    file_path = os.path.join(root, file)
                    with open(file_path, 'r', encoding='utf-8') as f:
                        lines = f.readlines()

                    with open(file_path, 'w', encoding='utf-8') as f:
                        for line in lines:
                            for pattern in sleep_patterns:
                                match = pattern.match(line)
                                if match:
                                    line = f"{match.group(1)}# {line}"
                                    break
                            f.write(line)

    def run_multiple_experiments(self, experiment_list):
        if not experiment_list:
            experiment_list = sorted(self.get_experiment_directories())
        try:
            with tqdm(total=len(experiment_list), desc="Running Experiments") as pbar:
                for experiment in experiment_list:
                    if self.script_name == 'run_meta.py':
                        self.setup_metagpt(os.path.join(f"../workspace/{self.experiment_path}", experiment))
                    if self.script_name == 'run_cap.py':
                        self.setup_cap(os.path.join(f"../workspace/{self.experiment_path}", experiment))
                    self.stop_event.clear()
                    result_queue = queue.Queue()

                    t1 = threading.Thread(target=self.run_code, args=(experiment, self.script_name, result_queue))
                    t2 = threading.Thread(target=self.run_single_experiment, args=(experiment, result_queue))

                    t1.start()
                    t2.start()

                    t1.join(timeout=self.experiment_duration - 1)
                    t2.join(timeout=self.experiment_duration - 1)

                    if t1.is_alive() or t2.is_alive():
                        self.stop_event.set()
                        t1.join()
                        t2.join()
                    single_experiment_result = None
                    run_code_result = None
                    while not result_queue.empty():
                        result = result_queue.get()
                        if result['source'] == 'run_single_experiment':
                            single_experiment_result = result['result']
                        elif result['source'] == 'run_code':
                            run_code_result = result

                    analysis = self.analyze_result(single_experiment_result)
                    print(f"Analysis for experiment {experiment}: {analysis}")
                    self.results[experiment] = analysis
                    self.save_experiment_result(os.path.join(f"../workspace/{self.experiment_path}", experiment),
                                                single_experiment_result, analysis, run_code_result)
                    pbar.update(1)

        except KeyboardInterrupt:
            print("Keyboard interrupt received. Stopping all experiments.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.stop_event.set()
            t1.join()
            t2.join()

        print("All experiments completed successfully.")

    def plot_and_print_results(self, data, labels, ylabel, title, colors, save_filename, rotation=False):
        if rotation:
            rotation = -90
        else:
            rotation = 0

        plt.figure(figsize=(10, 6))
        plt.bar(labels, data, color=colors)
        plt.xticks(rotation=rotation)
        plt.ylabel(ylabel)
        plt.title(title)
        for i, v in enumerate(data):
            plt.text(i, v + 0.01, f"{v:.2f}", ha='center', rotation=rotation)
        if not os.path.exists(f"../workspace/{self.experiment_path}/pic"):
            os.makedirs(f"../workspace/{self.experiment_path}/pic")
        plt_path = os.path.join(f"../workspace/{self.experiment_path}/pic", save_filename)
        plt.tight_layout()
        plt.savefig(plt_path)
        plt.show()
        print(f"{title}: {data}")

    def analyze_code(self, functions_path):
        from modules.utils.function import CodeAnalyzer
        with open(functions_path, 'r') as f:
            analyzer = CodeAnalyzer(f.read())
            analysis_result = analyzer.analyze()
        # TODO:add more metrics
        return analysis_result["code_lines"]

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
    runner = AutoRunner("../config/env_config.json",
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

    if runner.run_mode in ['rerun', 'continue']:
        exp_list = sorted(extra_exp(f"../workspace/{runner.experiment_path}", out_type='name'))
        runner.run_multiple_experiments(exp_list)

    if runner.run_mode == 'analyze':
        exp_list = sorted(extra_exp(f"../workspace/{runner.experiment_path}", out_type='name'))

        runner.analyze_all_results(exp_list)
