import math
import os
import threading
import subprocess
import rospy
import json
import numpy as np
from tqdm import tqdm

from modules.deployment.gymnasium_env import GymnasiumCrossEnvironment
from modules.deployment.utils.manager import Manager


class AutoRunner:
    def __init__(self, env_config_path,
                 workspace_path,
                 experiment_duration,
                 run_mode='rerun',
                 max_speed=1.0):
        self.env_config_path = env_config_path
        self.experiment_path = workspace_path
        self.experiment_duration = experiment_duration
        self.env = GymnasiumCrossEnvironment(self.env_config_path)
        self.env.reset()
        self.results = {}
        self.manager = Manager(self.env, max_speed=max_speed)
        self.stop_event = threading.Event()
        self.run_mode = run_mode

    def get_experiment_directories(self):
        directories = []
        for item in os.listdir(f"../workspace/{self.experiment_path}"):
            item_path = os.path.join(f"../workspace/{self.experiment_path}", item)
            if os.path.isdir(item_path):
                if self.run_mode == 'continue':
                    if self.experiment_completed(item_path):
                        directories.append(item)
                elif self.run_mode == 'rerun':
                    directories.append(item)
        return directories

    def experiment_completed(self, path):
        # 检查是否已经存在 JSON 文件
        result_file = os.path.join(path, 'result.json')
        return os.path.exists(result_file)

    def save_experiment_result(self, path, result, analysis):
        # 保存结果为 JSON 文件
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
            'analysis': serializable_analysis,
            'experiment_data': serializable_result,
        }
        result_file = os.path.join(path, 'result.json')
        os.makedirs(path, exist_ok=True)
        with open(result_file, 'w') as f:
            json.dump(combined_result, f, indent=4)

    def run_single_experiment(self, experiment_id):
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
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            current_time = rospy.get_time()
            if current_time - start_time > self.experiment_duration:
                break
            action = self.manager.robotID_velocity
            self.manager.clear_velocity()
            obs, reward, termination, truncation, infos = self.env.step(action=action)
            for entity_id in infos:
                result[entity_id]["trajectory"].append(infos[entity_id]["position"])
            self.env.render()
            self.manager.publish_observations(infos)
            rate.sleep()
        print(f"Experiment {experiment_id} completed successfully.")
        return result

    def analyze_result(self, run_result):
        def are_trajectories_equal_length(data):
            # 检查所有 trajectory 长度是否相等
            lengths = [len(info["trajectory"]) for info in data.values()]
            return all(length == lengths[0] for length in lengths)

        def collision_check(data, tolerance=0.1):
            # 碰撞检查的逻辑
            num_timesteps = len(next(iter(data.values()))["trajectory"])
            for t in range(num_timesteps):
                for id1, info1 in data.items():
                    for id2, info2 in data.items():
                        if id1 >= id2:
                            continue
                        pos1 = info1["trajectory"][t]
                        pos2 = info2["trajectory"][t]
                        distance = math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
                        if distance + tolerance < (info1["size"] + info2["size"]):
                            return True
            return False

        def target_achieve(data, tolerance=0.1):
            # 检查是否在目标点或附近
            for entity_id, info in data.items():
                target = info["target"]
                if target is None:
                    continue
                final_position = info["trajectory"][-1]
                distance = math.sqrt((target[0] - final_position[0]) ** 2 + (target[1] - final_position[1]) ** 2)
                if distance > tolerance:
                    return False
            return True

        if not are_trajectories_equal_length(run_result):
            raise ValueError("Trajectories have different lengths")

        return {
            'collision': collision_check(run_result),
            'target_achieve': target_achieve(run_result)
        }

    def run_and_analyze_experiment(self, experiment):
        result = self.run_single_experiment(experiment)
        analysis = self.analyze_result(result)
        print(f"Analysis for experiment {experiment}: {analysis}")
        self.results[experiment] = analysis
        self.save_experiment_result(os.path.join(f"../workspace/{self.experiment_path}", experiment), result, analysis)

    def run_code(self, experiment):
        experiment_path = os.path.join(self.experiment_path, experiment)
        command = ['python', '../modules/framework/actions/run_code.py', '--data', experiment_path, '--timeout',
                   str(self.experiment_duration - 3)]

        try:
            result = subprocess.run(command, timeout=self.experiment_duration - 2, capture_output=True, text=True,
                                    check=True)
            # print(result.stdout)
        except subprocess.TimeoutExpired:
            print(f"\nExperiment {experiment} timed out and was terminated.")
        except subprocess.CalledProcessError as e:
            print(f"An error occurred while running {experiment}: {e}")
            # print(f"Output: {e.stdout}")
            print(f"Errors: {e.stderr}")

    def run_multiple_experiments(self):
        experiment_list = self.get_experiment_directories()
        try:
            with tqdm(total=len(experiment_list), desc="Running Experiments") as pbar:
                for experiment in experiment_list:
                    self.stop_event.clear()

                    t1 = threading.Thread(target=self.run_code, args=(experiment,))
                    t2 = threading.Thread(target=self.run_and_analyze_experiment, args=(experiment,))

                    t1.start()
                    t2.start()

                    t1.join(timeout=self.experiment_duration - 1)
                    t2.join(timeout=self.experiment_duration - 1)

                    if t1.is_alive() or t2.is_alive():
                        self.stop_event.set()
                        t1.join()
                        t2.join()

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


if __name__ == "__main__":
    runner = AutoRunner("../config/env_config.json",
                        workspace_path='layer/cross',
                        experiment_duration=40,
                        run_mode='continue'
                        )
    runner.run_multiple_experiments()
