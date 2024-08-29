import numpy as np
from itertools import combinations

from scipy.spatial import Delaunay, procrustes

from run.utils import calculate_overlap_ratio


def check_collisions(data, tolerance: float = 0.1) -> dict:
    """
    Check if there are any collisions between robots and obstacles at any time step.
    :param data: dictionary containing information about entities
    :param tolerance: allowed tolerance for collision
    :return:
        a dictionary containing:
        - collision (bool): Whether any collisions occurred.
        - collision_count (int): The total number of collisions across all time steps.
        - collision_severity_sum (float): The sum of overlap ratios for all collisions.
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
                    robot_position, obstacle_info["trajectory"][0], robot_info["size"], obstacle_info["size"], tolerance
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
    return {"collision": collision,
            "collision_count": collision_count,
            "collision_severity_sum": collision_severity_sum}


def evaluate_target_achievement(data, tolerance=0.1) -> dict:
    """
    Evaluate the performance of entities in achieving their targets.

    :param data: dictionary containing information about entities, including their trajectories and targets.
    :param tolerance: the distance threshold within which a target is considered achieved.
    :return: A dict containing:
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

    return {
        "all_targets_achieved": all_targets_achieved,
        "target_achievement_ratio": target_achievement_ratio,
        "average_distance_ratio": average_distance_ratio,
        "average_steps_ratio": average_steps_ratio
    }


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


def calculate_line_similarity(data, target_line: tuple) -> dict:
    """
    Calculate the similarity between the line formed by the robots' final positions and a given target line.

    :param data: dictionary containing information about entities, including their trajectories.
    :param target_line: a tuple containing two points (x1, y1) and (x2, y2) defining the target line.
    :return: A dictionary containing:
        - length_similarity (float): The ratio of the lengths between the fitted line and the target line.
        - slope_similarity (float): The cosine similarity between the slopes of the fitted line and the target line.
        - point_similarity (float): The average Euclidean distance between the points on the fitted line and the target line.
    """
    robot_positions = []
    for entity_id, info in data.items():
        if info["type"] == "Robot":
            robot_positions.append(info["trajectory"][-1])  # Get final positions of robots

    if len(robot_positions) < 2:
        raise ValueError("Not enough robots to form a line.")

    # Fit a line to the robots' final positions
    robot_positions = np.array(robot_positions)
    A = np.vstack([robot_positions[:, 0], np.ones(len(robot_positions))]).T
    slope, intercept = np.linalg.lstsq(A, robot_positions[:, 1], rcond=None)[0]

    # Fitted line: y = slope * x + intercept
    fitted_line = ((0, intercept), (1, slope + intercept))

    # Calculate length similarity
    target_length = np.linalg.norm(np.array(target_line[1]) - np.array(target_line[0]))
    fitted_length = np.linalg.norm(np.array(fitted_line[1]) - np.array(fitted_line[0]))
    length_similarity = min(fitted_length / target_length, target_length / fitted_length) if target_length > 0 else 0

    # Calculate slope similarity (using cosine similarity)
    target_slope = (target_line[1][1] - target_line[0][1]) / (target_line[1][0] - target_line[0][0]) if target_line[1][
                                                                                                            0] != \
                                                                                                        target_line[0][
                                                                                                            0] else np.inf
    slope_similarity = np.dot([1, slope], [1, target_slope]) / (
            np.linalg.norm([1, slope]) * np.linalg.norm([1, target_slope]))

    # Calculate point similarity (average distance between points on the fitted line and target line)
    point_distances = []
    for point in robot_positions:
        target_y = target_slope * point[0] + (target_line[0][1] - target_slope * target_line[0][0])
        fitted_y = slope * point[0] + intercept
        point_distances.append(np.abs(target_y - fitted_y))
    point_similarity = np.mean(point_distances)

    return {
        "length_similarity": length_similarity,
        "slope_similarity": slope_similarity,
        "point_similarity": point_similarity
    }


def evaluate_robot_final_positions(data) -> dict:
    """
    Evaluate various metrics related to the final positions of robots.

    :param data: dictionary containing information about entities, including their trajectories.
    :return: A dictionary containing:
        - mean_nearest_neighbor_distance (float): The mean distance between each robot and its nearest neighbor.
        - variance_nearest_neighbor_distance (float): The variance of the distances between each robot and its nearest neighbor.
        - mean_displacement (float): The mean displacement of robots from their initial positions to their final positions.
        - area_ratio (float): The ratio of the area occupied by the robots' final positions to 25.
    """
    robot_positions_initial = []
    robot_positions_final = []

    for entity_id, info in data.items():
        if info["type"] == "Robot":
            robot_positions_initial.append(info["trajectory"][0])
            robot_positions_final.append(info["trajectory"][-1])

    robot_positions_initial = np.array(robot_positions_initial)
    robot_positions_final = np.array(robot_positions_final)

    # Calculate nearest neighbor distances for final positions
    nearest_neighbor_distances = []
    for i, pos1 in enumerate(robot_positions_final):
        distances = np.linalg.norm(robot_positions_final - pos1, axis=1)
        distances[i] = np.inf  # Exclude the distance to itself
        nearest_neighbor_distances.append(np.min(distances))

    mean_nearest_neighbor_distance = np.mean(nearest_neighbor_distances)
    variance_nearest_neighbor_distance = np.var(nearest_neighbor_distances)

    # Calculate mean displacement from initial to final positions
    displacements = np.linalg.norm(robot_positions_final - robot_positions_initial, axis=1)
    mean_displacement = np.mean(displacements)

    # Calculate the area of the bounding box formed by the robots' final positions
    x_min, y_min = np.min(robot_positions_final, axis=0)
    x_max, y_max = np.max(robot_positions_final, axis=0)
    area = (x_max - x_min) * (y_max - y_min)
    area_ratio = area / 25

    return {
        "mean_nearest_neighbor_distance": mean_nearest_neighbor_distance,
        "variance_nearest_neighbor_distance": variance_nearest_neighbor_distance,
        "mean_displacement": mean_displacement,
        "area_ratio": area_ratio
    }


def evaluate_robot_circle_similarity(data, circle_center: tuple, circle_radius: float) -> dict:
    """
    Evaluate the similarity between the robots' final positions and a specified circle.

    :param data: dictionary containing information about entities, including their trajectories.
    :param circle_center: a tuple representing the (x, y) coordinates of the circle's center.
    :param circle_radius: the radius of the circle.
    :return: A dictionary containing:
        - mean_distance_to_circle (float): The mean distance of each robot's final position to the circle's circumference.
        - variance_distance_to_circle (float): The variance of the distances to the circle's circumference.
        - mean_nearest_neighbor_distance (float): The mean distance between each robot and its nearest neighbor.
        - dispersion_ratio (float): The ratio of the circle's circumference to the number of robots, normalized by the mean nearest neighbor distance.
    """
    robot_positions_final = []

    for entity_id, info in data.items():
        if info["type"] == "Robot":
            robot_positions_final.append(info["trajectory"][-1])

    robot_positions_final = np.array(robot_positions_final)
    num_robots = len(robot_positions_final)

    if num_robots == 0:
        raise ValueError("No robots found in the data.")

    # Calculate distance of each robot to the circle's circumference
    distances_to_circle = []
    for position in robot_positions_final:
        distance_to_center = np.linalg.norm(position - np.array(circle_center))
        distance_to_circle = np.abs(distance_to_center - circle_radius)
        distances_to_circle.append(distance_to_circle)

    mean_distance_to_circle = np.mean(distances_to_circle)
    variance_distance_to_circle = np.var(distances_to_circle)

    # Calculate nearest neighbor distances for final positions
    nearest_neighbor_distances = []
    for i, pos1 in enumerate(robot_positions_final):
        distances = np.linalg.norm(robot_positions_final - pos1, axis=1)
        distances[i] = np.inf  # Exclude the distance to itself
        nearest_neighbor_distances.append(np.min(distances))

    mean_nearest_neighbor_distance = np.mean(nearest_neighbor_distances)

    # Calculate the dispersion ratio (circumference / num_robots) normalized by mean nearest neighbor distance
    circumference = 2 * np.pi * circle_radius
    dispersion_ratio = (
                               circumference / num_robots) / mean_nearest_neighbor_distance if mean_nearest_neighbor_distance > 0 else np.inf

    return {
        "mean_distance_to_circle": mean_distance_to_circle,
        "variance_distance_to_circle": variance_distance_to_circle,
        "mean_nearest_neighbor_distance": mean_nearest_neighbor_distance,
        "dispersion_ratio": dispersion_ratio
    }


def evaluate_trajectory_pattern(data) -> dict:
    """
    Evaluate the trajectory pattern by analyzing the spatial distribution and overall shape of robot final positions.

    :param data: dictionary containing information about entities, including their trajectories.
    :return: A dictionary containing:
        - spatial_variance (float): Variance of the robot positions indicating the spread.
        - bounding_box_area (float): Area of the bounding box enclosing all final positions of the robots.
    """
    robot_positions_final = []

    for entity_id, info in data.items():
        if info["type"] == "Robot":
            robot_positions_final.append(info["trajectory"][-1])  # Use the final position of each robot

    robot_positions_final = np.array(robot_positions_final)
    num_robots = len(robot_positions_final)

    if num_robots == 0:
        raise ValueError("No robots found in the data.")

    # Calculate the spatial variance of robot positions
    spatial_variance = np.var(robot_positions_final, axis=0).sum()

    # Calculate the area of the bounding box formed by the robots' final positions
    x_min, y_min = np.min(robot_positions_final, axis=0)
    x_max, y_max = np.max(robot_positions_final, axis=0)
    bounding_box_area = (x_max - x_min) * (y_max - y_min)

    return {
        "spatial_variance": spatial_variance,
        "bounding_box_area": bounding_box_area
    }


def evaluate_trajectory_similarity(data) -> dict:
    """
    Evaluate the similarity between the trajectories of all robots using Dynamic Time Warping (DTW).

    :param data: dictionary containing information about entities, including their trajectories.
    :return: A dictionary containing:
        - mean_dtw_distance (float): The mean DTW distance between all pairs of robot trajectories.
        - variance_dtw_distance (float): The variance of DTW distances between all pairs of robot trajectories.
        - max_dtw_distance (float): The maximum DTW distance between any pair of robot trajectories.
        - min_dtw_distance (float): The minimum DTW distance between any pair of robot trajectories.
    """
    from fastdtw import fastdtw
    from scipy.spatial.distance import euclidean
    trajectories = []

    for entity_id, info in data.items():
        if info["type"] == "Robot":
            trajectories.append(np.array(info["trajectory"]))

    num_robots = len(trajectories)

    if num_robots < 2:
        raise ValueError("Not enough robots to calculate trajectory similarity.")

    dtw_distances = []

    for i in range(num_robots):
        for j in range(i + 1, num_robots):
            distance, _ = fastdtw(trajectories[i], trajectories[j], dist=euclidean)
            dtw_distances.append(distance)

    mean_dtw_distance = np.mean(dtw_distances)
    variance_dtw_distance = np.var(dtw_distances)
    max_dtw_distance = np.max(dtw_distances)
    min_dtw_distance = np.min(dtw_distances)

    return {
        "mean_dtw_distance": mean_dtw_distance,
        "variance_dtw_distance": variance_dtw_distance,
        "max_dtw_distance": max_dtw_distance,
        "min_dtw_distance": min_dtw_distance
    }


def evaluate_shape_similarity(data, target_shape: list) -> dict:
    """
    Evaluate the similarity between the robots' final positions and a specified shape using both
    traditional metrics and Procrustes analysis.

    :param data: dictionary containing information about entities, including their trajectories.
    :param target_shape: a list of tuples representing the vertices of the target shape in order.
    :return: A dictionary containing:
        - mean_distance_to_shape (float): The mean distance of each robot's final position to the nearest point on the target shape.
        - variance_distance_to_shape (float): The variance of the distances to the nearest point on the target shape.
        - shape_coverage_ratio (float): The ratio of the number of robots' final positions that fall within the shape's boundaries to the total number of robots.
        - procrustes_distance (float): A measure of dissimilarity between the robot positions and the target shape using Procrustes analysis.
    """
    robot_positions_final = []

    for entity_id, info in data.items():
        if info["type"] == "Robot":
            robot_positions_final.append(info["trajectory"][-1])

    robot_positions_final = np.array(robot_positions_final)

    if len(robot_positions_final) == 0:
        raise ValueError("No robots found in the data.")

    # Traditional Metrics Calculation
    target_shape = np.array(target_shape)
    shape_delaunay = Delaunay(target_shape)

    distances_to_shape = []
    within_shape_count = 0

    for position in robot_positions_final:
        # Calculate the distance to the nearest point on the target shape
        min_distance = np.min([np.linalg.norm(position - vertex) for vertex in target_shape])
        distances_to_shape.append(min_distance)

        # Check if the position is inside the shape's convex hull using Delaunay
        if shape_delaunay.find_simplex(position) >= 0:
            within_shape_count += 1

    mean_distance_to_shape = np.mean(distances_to_shape)
    variance_distance_to_shape = np.var(distances_to_shape)
    shape_coverage_ratio = within_shape_count / len(robot_positions_final) if len(robot_positions_final) > 0 else 0

    # Procrustes Analysis
    if len(robot_positions_final) != len(target_shape):
        raise ValueError(
            "The number of robot positions and the target shape points must be equal for Procrustes analysis.")

    mtx1, mtx2, disparity = procrustes(target_shape, robot_positions_final)

    return {
        "mean_distance_to_shape": mean_distance_to_shape,
        "variance_distance_to_shape": variance_distance_to_shape,
        "shape_coverage_ratio": shape_coverage_ratio,
        "procrustes_distance": disparity
    }


def evaluate_encircling(data, target_radius: float = 0.5, tolerance: float = 0.1) -> dict:
    """
    Evaluate the task completion by calculating the mean and variance of the robots' final distances to the prey,
    and the number of steps taken to get within a specified range of the target distance.

    :param data: dictionary containing information about entities, including their trajectories.
    :param target_radius: the target distance from the prey's position (default is 0.5).
    :param tolerance: the distance tolerance within which the target is considered achieved.
    :return: A dictionary containing:
        - mean_distance_error (float): The mean absolute error of the robots' final distance to the target radius.
        - variance_distance_error (float): The variance of the distance errors.
        - initial_distance_error (float): The initial mean distance to the target radius.
        - steps_to_target_range (int): The number of steps taken until the mean distance first falls within the target range.
    """
    robot_positions_initial = []
    robot_positions_final = []
    initial_distances = []
    final_distances = []
    steps_to_target_range = float('inf')
    within_tolerance_found = False

    prey_trajectory = None

    # First pass: find the prey trajectory
    for entity_id, info in data.items():
        if info["type"] == "Prey":
            prey_trajectory = np.array(info["trajectory"])
            break

    if prey_trajectory is None:
        raise ValueError("Prey trajectory not found in data.")

    # Second pass: process robot data
    for entity_id, info in data.items():
        if info["type"] == "Robot":
            trajectory = np.array(info["trajectory"])
            initial_distances.append(np.linalg.norm(trajectory[0] - prey_trajectory[0]))
            robot_positions_initial.append(trajectory[0])
            robot_positions_final.append(trajectory[-1])

    if prey_trajectory is None:
        raise ValueError("Prey trajectory not found in data.")

    final_distances = []
    mean_distances = []

    for t in range(len(prey_trajectory)):
        distances = [np.linalg.norm(robot_pos - prey_trajectory[t]) for robot_pos in robot_positions_final]
        final_distances.append(distances)
        mean_distances.append(np.mean(distances))

        if not within_tolerance_found:
            if abs(mean_distances[-1] - target_radius) <= tolerance:
                steps_to_target_range = t + 1
                within_tolerance_found = True

    mean_distance_error = np.mean([abs(distance - target_radius) for distance in mean_distances])
    variance_distance_error = np.var([abs(distance - target_radius) for distance in mean_distances])

    initial_mean_distance = np.mean(initial_distances)

    return {
        "mean_distance_error": mean_distance_error,
        "variance_distance_error": variance_distance_error,
        "initial_distance_error": abs(initial_mean_distance - target_radius),
        "steps_to_target_range": steps_to_target_range if within_tolerance_found else 'inf'
    }
