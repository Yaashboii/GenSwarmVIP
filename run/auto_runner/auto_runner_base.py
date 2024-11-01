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

import json
import operator
import os
import queue
import time
import traceback
from abc import ABC, abstractmethod

import numpy as np
from tqdm import tqdm
from modules.deployment.gymnasium_env import GymnasiumEnvironmentBase
from modules.utils import root_manager
from run.auto_runner.core import CodeRunner, ExperimentAnalyzer, EnvironmentManager
from run.utils import setup_metagpt, setup_cap, check_robots_no_movement_in_first_third


class AutoRunnerBase(ABC):
    def __init__(
        self,
        env_config_path,
        workspace_path,
        experiment_duration,
        run_mode="rerun",
        target_pkl="WriteRun.pkl",
        script_name="run.py",
        exp_batch=1,
        max_speed=1.0,
        tolerance=0.05,
        env: GymnasiumEnvironmentBase = None,
    ):
        self.exp_batch = exp_batch
        self.env_config_path = env_config_path
        self.experiment_path = root_manager.project_root / f"workspace/{workspace_path}"
        self.experiment_duration = experiment_duration
        self.results = {}
        self.target_pkl = target_pkl
        self.script_name = script_name
        self.run_mode = run_mode
        self.success_conditions = self.setup_success_conditions()
        self.result_analyzer = ExperimentAnalyzer(
            experiment_path=self.experiment_path,
            success_conditions=self.success_conditions,
        )
        self.tolerance = tolerance
        self.sim_env = EnvironmentManager(env, max_speed=max_speed)
        self.code_runner = CodeRunner(
            time_out=experiment_duration,
            target_pkl=target_pkl,
            script_name=script_name,
            feedback="None",
            experiment_path=self.experiment_path,
            env_manager=self.sim_env,
        )

    def get_experiment_directories(self):
        directories = []
        for item in os.listdir(self.experiment_path):
            if item == "pic" or item == "real":
                continue
            item_path = os.path.join(self.experiment_path, item)
            if os.path.isdir(item_path):
                if self.run_mode == "continue":
                    if not self.experiment_completed(item_path):
                        directories.append(item)
                elif self.run_mode == "rerun":
                    directories.append(item)
                elif self.run_mode == "analyze":
                    if self.experiment_completed(item_path):
                        directories.append(item)
                elif self.run_mode == "fail_rerun":
                    if self.experiment_completed(item_path):
                        result_file = os.path.join(item_path, "wo_vlm.json")
                        with open(result_file, "r") as f:
                            result_data = json.load(f)

                            analysis = result_data.get("analysis", {})
                            # bug = result_data.get('run_code_result', {})['error']
                            success = analysis["success"]
                            if not success:
                                directories.append(item)
        if self.run_mode == "rerun" or self.run_mode == "fail_rerun":
            # 根据batch数目，将实验分批次,一共10批，根据batch数目，确定当前分批
            directories = sorted(directories)
            batch_size = len(directories) // 10
            batch_num = self.exp_batch
            directories = directories[
                (batch_num - 1) * batch_size : batch_num * batch_size
            ]
            print(
                f"Batch {batch_num} from {batch_size * (batch_num - 1)} to {batch_size * batch_num}"
            )
        return directories

    def experiment_completed(self, path):
        result_file = os.path.join(path, "wo_vlm.json")
        return os.path.exists(result_file)

    def save_experiment_result(self, path, result, analysis, file_name=""):
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
            "analysis": serializable_analysis,
            "experiment_data": serializable_result,
        }
        result_file = os.path.join(path, file_name)
        os.makedirs(path, exist_ok=True)
        with open(result_file, "w") as f:
            json.dump(combined_result, f, indent=4)

    def run_multiple_experiments(self, experiment_list):
        if not experiment_list:
            experiment_list = sorted(self.get_experiment_directories())

        try:
            with tqdm(total=len(experiment_list), desc="Running Experiments") as pbar:
                for experiment in experiment_list:
                    retries = 0
                    max_retries = 1
                    success = False

                    while retries < max_retries and not success:
                        if self.script_name == "run_meta.py":
                            setup_metagpt(
                                os.path.join(self.experiment_path, experiment)
                            )
                        if self.script_name == "run_cap.py":
                            setup_cap(os.path.join(self.experiment_path, experiment))

                        self.code_runner.run_code(experiment)
                        # load result from file
                        wo_vlm_result, vlm_result = self.code_runner.load_result(
                            experiment
                        )
                        if wo_vlm_result is not None:
                            wo_vlm_analysis = self.analyze_result(wo_vlm_result)
                            wo_experiment_success = (
                                self.result_analyzer.calculate_success(wo_vlm_analysis)
                            )
                            success_dict = {"success": wo_experiment_success}
                            wo_vlm_analysis.update(success_dict)

                        else:
                            wo_vlm_analysis = {"success": False}
                        # if vlm_result is not None:
                        #     vlm_analysis = self.analyze_result(vlm_result)
                        #     vlm_experiment_success = self.result_analyzer.calculate_success(vlm_analysis)
                        #     success_dict = {"success": vlm_experiment_success}
                        #     vlm_analysis.update(success_dict)
                        # else:
                        #     vlm_analysis = {"success": False}
                        # 检查是否有未移动的机器人
                        unmoved_robots = check_robots_no_movement_in_first_third(
                            wo_vlm_result
                        )

                        if not unmoved_robots:
                            print(f"Experiment {experiment} completed successfully.")
                            success = True  # 实验成功，跳出重试循环
                        else:
                            retries += 1
                            if retries < max_retries:
                                print(
                                    f"Experiment {experiment} failed (Unmoved robots or unsuccessful), retrying... (Attempt {retries + 1})"
                                )
                            time.sleep(2)  # 等待3秒后重新开始实验

                    print(
                        f"Analysis for wo_experiment {experiment}: {wo_vlm_analysis},\n"
                    )

                    self.save_experiment_result(
                        os.path.join(self.experiment_path, experiment),
                        wo_vlm_result,
                        wo_vlm_analysis,
                        file_name="wo_vlm.json",
                    )
                    # self.save_experiment_result(
                    #     os.path.join(self.experiment_path, experiment),
                    #     vlm_result, vlm_analysis, file_name='vlm.json')

                    print(f"Experiment {experiment} completed successfully.")

                    pbar.update(1)

        except KeyboardInterrupt:
            print("Keyboard interrupt received. Stopping all experiments.")
        except Exception as e:
            traceback.print_exc()
            print(f"An error occurred: {e}")

    def run(self, exp_list=None):
        if self.run_mode in ["rerun", "continue", "fail_rerun"]:
            self.run_multiple_experiments(exp_list)

        if self.run_mode == "analyze":
            if exp_list is None:
                exp_list = sorted(self.get_experiment_directories())
            self.result_analyzer.analyze_all_results(exp_list)

    @abstractmethod
    def analyze_result(self, run_result) -> dict[str, float]:
        """
        Analyze the result of a single experiment and return a dictionary of metrics.
        """
        raise NotImplementedError(
            "analyze_result method must be implemented in the subclass"
        )

    @abstractmethod
    def setup_success_conditions(self) -> list[tuple[str, operator, float]]:
        """
        This method should be implemented in the subclass to set up the success conditions for the experiments.
        The success conditions should be a list of tuples, where each tuple contains the metric name, the comparison
        operator, and the threshold value. For example, to set up success conditions for a metric "mean_dtw_distance"
        with a threshold of 500, the method should return a list like this:
        [("mean_dtw_distance", operator.lt, 500)]
        """
        raise NotImplementedError(
            "setup_success_conditions method must be implemented in the subclass"
        )
