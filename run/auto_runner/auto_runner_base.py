import json
import os
import queue
import subprocess
import threading
from abc import ABC, abstractmethod

import cv2
import imageio
import matplotlib.pyplot as plt
import numpy as np
import rospy
from tqdm import tqdm

from modules.deployment.gymnasium_env import GymnasiumEnvironmentBase
from modules.deployment.utils.manager import Manager
from run.utils import setup_metagpt, setup_cap


class AutoRunnerBase(ABC):
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 target_pkl='WriteRun.pkl',
                 script_name='run.py',
                 max_speed=1.0,
                 tolerance=0.05,
                 env: GymnasiumEnvironmentBase = None):
        self.env_config_path = env_config_path
        self.experiment_path = workspace_path
        self.experiment_duration = experiment_duration
        self.env = env
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

    def run_single_experiment(self, experiment_id, result_queue):
        obs, infos = self.env.reset()

        def init_result(infos):
            result = {}
            for entity_id in infos:
                result[entity_id] = {"size": 0,
                                     "target": None,
                                     "trajectory": [],
                                     'type': '',
                                     'states': [],
                                     'dt': self.env.dt}
                result[entity_id]["size"] = infos[entity_id]["size"]
                result[entity_id]["target"] = infos[entity_id]["target_position"]
                result[entity_id]['type'] = infos[entity_id]['type']
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
                if infos[entity_id]["moveable"]:
                    result[entity_id]["trajectory"].append(infos[entity_id]["position"])
                if infos[entity_id]['state'] is not None:
                    result[entity_id]['states'].append(infos[entity_id]['state'])
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

    def run_multiple_experiments(self, experiment_list):
        if not experiment_list:
            experiment_list = sorted(self.get_experiment_directories())
        try:
            with tqdm(total=len(experiment_list), desc="Running Experiments") as pbar:
                for experiment in experiment_list:
                    if self.script_name == 'run_meta.py':
                        setup_metagpt(os.path.join(f"../workspace/{self.experiment_path}", experiment))
                    if self.script_name == 'run_cap.py':
                        setup_cap(os.path.join(f"../workspace/{self.experiment_path}", experiment))
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
        from modules.utils import CodeAnalyzer
        with open(functions_path, 'r') as f:
            analyzer = CodeAnalyzer(f.read())
            analysis_result = analyzer.analyze()
        # TODO:add more metrics
        return analysis_result["code_lines"]

    def run(self, exp_list=None):
        if self.run_mode in ['rerun', 'continue']:
            self.run_multiple_experiments(exp_list)

        if self.run_mode == 'analyze':
            self.analyze_all_results(exp_list)

    @abstractmethod
    def analyze_result(self, run_result) -> dict[str, float]:
        """
        Analyze the result of a single experiment and return a dictionary of metrics.
        """
        raise NotImplementedError("analyze_result method must be implemented in the subclass")

    @abstractmethod
    def analyze_all_results(self, experiment_dirs=None):
        """
        Analyze all the results of the experiments and show the metrics.
        """
