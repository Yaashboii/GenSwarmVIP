import numpy as np
from itertools import combinations

from run.utils import calculate_overlap_ratio


def check_collisions(data, tolerance: float = 0.1) -> tuple:
    """
    Check if there are any collisions between robots and obstacles at any time step.
    :param data: dictionary containing information about entities
    :param tolerance: allowed tolerance for collision
    :return:
    - int: number of collisions
    - float: sum of severity of all collisions
    """
    robots = {id: info for id, info in data.items() if info["type"] == "Robot"}
    obstacles = {id: info for id, info in data.items() if info["type"] == "Obstacle"}
    num_timesteps = len(next(iter(data.values()))["trajectory"])
    collision_count = 0
    collision_severity_sum = 0

    for t in range(num_timesteps):
        # Check for robot-obstacle collisions
        for robot_id, robot_info in robots.items():
            robot_position = robot_info["trajectory"][t]
            for _, obstacle_info in obstacles.items():
                overlap_ratio = calculate_overlap_ratio(
                    robot_position, obstacle_info["position"], robot_info["size"], obstacle_info["size"], tolerance
                )
                if overlap_ratio is not None:
                    collision_count += 1
                    collision_severity_sum += overlap_ratio

        # Check for robot-robot collisions
        for (id1, robot1_info), (id2, robot2_info) in combinations(robots.items(), 2):
            robot1_position = robot1_info["trajectory"][t]
            robot2_position = robot2_info["trajectory"][t]
            overlap_ratio = calculate_overlap_ratio(
                robot1_position, robot2_position, robot1_info["size"], robot2_info["size"], tolerance
            )
            if overlap_ratio is not None:
                collision_count += 1
                collision_severity_sum += overlap_ratio
    collision = collision_count > 0
    return collision, collision_count, collision_severity_sum


def evaluate_target_achievement(data, tolerance=0.1) -> tuple:
    """
    Evaluate the performance of entities in achieving their targets.

    :param data: dictionary containing information about entities, including their trajectories and targets.
    :param tolerance: the distance threshold within which a target is considered achieved.
    :return: A tuple containing:
        - all_targets_achieved (bool): Whether all targets were achieved by all entities.
        - target_achievement_ratio (float): The ratio of entities that achieved their targets.
        - average_distance_ratio (float): The average ratio of final distance to initial distance for all entities.
        - average_steps_ratio (float or str): The average ratio of steps taken to total steps available for entities that achieved their targets. Returns 'inf' if no targets were achieved.
    """
    total_distance_ratio = 0
    achieved_targets = 0
    num_targets = 0
    total_steps_ratio = 0

    for entity_id, info in data.items():
        target = info.get("target")
        if target is None:
            continue

        num_targets += 1
        initial_position = info["trajectory"][0]
        initial_distance = np.linalg.norm(target - initial_position)
        steps_to_target = len(info["trajectory"])
        final_distance = initial_distance

        for t, position in enumerate(info["trajectory"]):
            current_distance = np.linalg.norm(target - position)
            if current_distance <= tolerance:
                steps_to_target = t + 1
                final_distance = current_distance
                break

        if final_distance <= tolerance:
            achieved_targets += 1
            total_steps_ratio += steps_to_target / len(info["trajectory"])

        distance_ratio = final_distance / initial_distance if initial_distance > 0 else 1
        total_distance_ratio += distance_ratio

        print(f"Entity {entity_id}: Initial Distance: {initial_distance}, Final Distance: {final_distance}, "
              f"Ratio: {distance_ratio}, Steps to Target: {steps_to_target}")

    all_targets_achieved = achieved_targets == num_targets
    target_achievement_ratio = achieved_targets / num_targets if num_targets > 0 else 0
    average_distance_ratio = total_distance_ratio / num_targets if num_targets > 0 else 0
    average_steps_ratio = total_steps_ratio / achieved_targets if achieved_targets > 0 else 'inf'

    return all_targets_achieved, target_achievement_ratio, average_distance_ratio, average_steps_ratio


def evaluate_landmark_visits(data) -> tuple:
    """
    Evaluate the visitation of landmarks by entities.

    :param data: dictionary containing information about entities, including their types and states.
    :return: A tuple containing:
        - all_landmarks_visited (bool): Whether all landmarks were visited.
        - landmark_visit_ratio (float): The ratio of landmarks that were visited.
        - average_visit_step (float): The average time step at which landmarks were visited.
    """
    visited_landmarks = 0
    num_landmarks = 0
    visit_steps = []

    for entity_id, info in data.items():
        if info.get("type") == "Landmark":
            num_landmarks += 1
            if 'visited' in info.get('states', []):
                visited_landmarks += 1
                visit_step = info['states'].index('visited')
                print(f"Landmark {entity_id} visited at time step {visit_step}")
                visit_steps.append(visit_step)

    all_landmarks_visited = visited_landmarks == num_landmarks
    landmark_visit_ratio = visited_landmarks / num_landmarks if num_landmarks > 0 else 0
    average_visit_step = np.mean(visit_steps) if visit_steps else 0

    return all_landmarks_visited, landmark_visit_ratio, average_visit_step
