import json

import numpy as np
import pygame


def check_collision(position, size, shape, other):
    if shape == 'circle':
        if other.shape == 'circle':
            return np.linalg.norm(position - other.position) < (size + other.size)
        else:
            return check_circle_rectangle_collision(other, position, size)
    else:
        if other.shape == 'circle':
            return check_circle_rectangle_collision(other, position, size)
        else:
            rect1 = pygame.Rect(position[0] - size[0] / 2, position[1] - size[1] / 2, size[0], size[1])
            rect2 = pygame.Rect(other.position[0] - other.size[0] / 2, other.position[1] - other.size[1] / 2,
                                other.size[0], other.size[1])
            return rect1.colliderect(rect2)


def check_circle_rectangle_collision(rect, circle_position, circle_radius):
    circle_center = circle_position
    rect_center = rect.position
    rect_half_size = np.array(rect.size) / 2

    # Find the closest point to the circle within the rectangle
    closest_point = np.clip(circle_center, rect_center - rect_half_size, rect_center + rect_half_size)

    # Calculate the distance between the circle's center and this closest point
    distance = np.linalg.norm(circle_center - closest_point)

    return distance < circle_radius


def save_entities_to_file(entities, filename):
    entities_data = []
    for entity in entities:
        entity_data = {
            'type': entity.__class__.__name__,
            "id": entity.id,
            "position": entity.position.tolist(),
            "size": entity.size if isinstance(entity.size, float) else entity.size.tolist(),
            "color": entity.color,
            "moveable": entity.moveable,
            "collision": entity.collision
        }
        entities_data.append(entity_data)

    with open(filename, 'w') as file:
        json.dump(entities_data, file, indent=4)


def sample_points(center, num_points, min_distance, shape='circle', size=None, max_attempts_per_point=10):
    def distance(p1, p2):
        return np.sqrt(np.sum((p1 - p2) ** 2, axis=1))

    points = []
    center = np.array(center)
    attempts = 0

    if shape == 'circle':
        if size is None or len(size) != 1:
            raise ValueError("For circle, size should be a list or tuple with one element: [radius].")
        radius = size[0] - min_distance
    elif shape == 'rectangle':
        if size is None or len(size) != 2:
            raise ValueError("For rectangle, size should be a list or tuple with two elements: [width, height].")
        width, height = size
        width -= 2 * min_distance
        height -= 2 * min_distance
    else:
        raise ValueError(f"Unsupported shape: {shape}")

    while len(points) < num_points and attempts < num_points * max_attempts_per_point:
        if shape == 'circle':
            # Generate random points within the bounding square and then filter to within the circle
            random_points = np.random.uniform(-radius, radius, size=(num_points, 2)) + center
            valid_mask = np.linalg.norm(random_points - center, axis=1) <= radius
            random_points = random_points[valid_mask]
        elif shape == 'rectangle':
            # Generate random points within the bounding rectangle
            random_points = np.random.uniform(-0.5, 0.5, size=(num_points, 2)) * np.array([width, height]) + center
        else:
            raise ValueError(f"Unsupported shape: {shape}")

        for point in random_points:
            if len(points) == 0 or np.all(distance(np.array(points), point) >= min_distance):
                points.append(point)
                if len(points) >= num_points:
                    break

        attempts += 1

    if len(points) < num_points:
        raise Exception(f"Warning: Could only place {len(points)} points out of {num_points} requested.")

    return np.array(points)
