import math
import os
import json
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from modules.deployment.gymnasium_env import GymnasiumCrossingEnvironment
from run.auto_runner import AutoRunnerBase


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
        pass
        # def are_trajectories_equal_length(data):
        #     lengths = [len(info["trajectory"]) for info in data.values()]
        #     return all(length == lengths[0] for length in lengths)
        #
        # def collision_check(data, tolerance=0.1):
        #     target_entities = {id: info for id, info in data.items() if info["target"] is not None}
        #
        #     num_timesteps = len(next(iter(data.values()))["trajectory"])
        #     collision_count = 0
        #     collision_severity_sum = 0
        #
        #     processed_pairs = set()
        #
        #     for id1, info1 in target_entities.items():
        #         for t in range(num_timesteps):
        #             pos1 = info1["trajectory"][t]
        #             for id2, info2 in data.items():
        #                 if id1 == id2 or (id2, id1) in processed_pairs:
        #                     continue
        #                 pos2 = info2["trajectory"][t]
        #                 distance = math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
        #                 if distance + tolerance < (info1["size"] + info2["size"]):
        #                     collision_count += 1
        #                     overlap = info1["size"] + info2["size"] - distance
        #                     collision_severity_sum += overlap / (info1["size"] + info2["size"])
        #                     processed_pairs.add((id1, id2))
        #
        #     num_target_entities = len(target_entities)
        #     collision_frequency = collision_count / (
        #             num_target_entities * num_timesteps) if num_target_entities > 0 else 0
        #     collision_severity_mean = collision_severity_sum / collision_count if collision_count > 0 else 0
        #
        #     return collision_count > 0, collision_frequency, collision_severity_mean, collision_severity_sum
        #
        # def target_achieve(data, tolerance=0.1):
        #     total_distance_ratio = 0
        #     achieved_targets = 0
        #     num_targets = 0
        #     total_steps_ratio = 0
        #
        #     for entity_id, info in data.items():
        #         target = info["target"]
        #         if target is None:
        #             continue
        #         num_targets += 1
        #         initial_position = info["trajectory"][0]
        #         initial_distance = math.sqrt(
        #             (target[0] - initial_position[0]) ** 2 + (target[1] - initial_position[1]) ** 2)
        #
        #         steps_to_target = len(info["trajectory"])
        #         final_distance = initial_distance
        #         for t, position in enumerate(info["trajectory"]):
        #             current_distance = math.sqrt(
        #                 (target[0] - position[0]) ** 2 + (target[1] - position[1]) ** 2)
        #             if current_distance <= tolerance:
        #                 steps_to_target = t + 1
        #                 final_distance = current_distance
        #                 break
        #         if final_distance <= tolerance:
        #             achieved_targets += 1
        #             total_steps_ratio += steps_to_target / len(info["trajectory"])
        #         distance_ratio = final_distance / initial_distance if initial_distance > 0 else 1
        #         total_distance_ratio += distance_ratio
        #
        #         print(
        #             f"Entity {entity_id}: Initial Distance: {initial_distance}, Final Distance: {final_distance}, Ratio: {distance_ratio}, Steps to Target: {steps_to_target}")
        #
        #     target_achieve_ratio = achieved_targets / num_targets if num_targets > 0 else 0
        #     average_distance_ratio = total_distance_ratio / num_targets if num_targets > 0 else 0
        #     average_steps_ratio = total_steps_ratio / achieved_targets if achieved_targets > 0 else 'inf'
        #
        #     return achieved_targets == num_targets, target_achieve_ratio, average_distance_ratio, average_steps_ratio
        #
        # if not are_trajectories_equal_length(run_result):
        #     raise ValueError("Trajectories have different lengths")
        #
        # collision, collision_frequency, collision_severity_mean, collision_severity_sum = collision_check(
        #     run_result, tolerance=self.tolerance)
        # target_achieve, target_achieve_ratio, average_distance_ratio, average_steps_ratio = target_achieve(
        #     run_result, tolerance=2 * self.tolerance)
        #
        # return {
        #     'collision_occurred': collision,
        #     'collision_frequency': collision_frequency,
        #     'collision_severity_average': collision_severity_mean,
        #     'collision_severity_sum': collision_severity_sum,
        #     'target_achieved': target_achieve,
        #     'target_achievement_ratio': target_achieve_ratio,
        #     'distance_ratio_average': average_distance_ratio,
        #     'steps_ratio_average': average_steps_ratio
        # }




if __name__ == "__main__":
    runner = AutoRunnerCross("../../config/env/crossing_config.json",
                             workspace_path='ablation/human_feedback',
                             # workspace_path='metagpt',
                             # workspace_path='cap/cross',
                             experiment_duration=30,
                             run_mode='rerun',
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
