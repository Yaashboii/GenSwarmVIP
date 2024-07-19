import json

import numpy as np
import pygame


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

def sample_point(zone_center=(0, 0), zone_shape='circle', zone_size=None, robot_size=None, robot_shape: str = 'circle',
                 min_distance: float = 0, entities=None, max_attempts_per_point=1000):
    def distance(p1, p2):
        return np.sqrt(np.sum((p1 - p2) ** 2, axis=1))

    points = []
    center = np.array(zone_center)
    attempts = 0

    if zone_shape == 'circle':
        if zone_size is None or len(zone_size) != 1:
            raise ValueError("For circle, size should be a list or tuple with one element: [radius].")
        zone_radius = zone_size - min_distance - robot_size
    elif zone_shape == 'rectangle':
        if zone_size is None or len(zone_size) != 2:
            raise ValueError("For rectangle, size should be a list or tuple with two elements: [width, height].")
        zone_width, zone_height = zone_size
        zone_width -= min_distance + robot_size
        zone_height -= min_distance + robot_size
    else:
        raise ValueError(f"Unsupported shape: {zone_shape}")

    collide_flag = True
    while max_attempts_per_point and collide_flag == True:
        if zone_shape == 'circle':
            # Generate random points within the bounding rectangle and then filter to within the circle
            new_random_point = np.random.uniform(-zone_radius, zone_radius, size=(2,)) + center
            valid_mask = np.linalg.norm(new_random_point - center, axis=1) <= zone_radius
            new_random_point = new_random_point[valid_mask]
        elif zone_shape == 'rectangle':
            # Generate random points within the bounding rectangle
            new_random_point = np.random.uniform(-0.5, 0.5, size=(2,)) * np.array([zone_width, zone_height]) + center
        else:
            raise ValueError(f"Unsupported shape: {zone_shape}")

        collide_flag = check_collide(new_random_point, robot_size + min_distance, robot_shape, entities)

        max_attempts_per_point -= 1

    return np.array(new_random_point)

def check_collide(position, size, shape, entities):
    for entity in entities:
        if shape == 'circle':
            if entity.shape == 'circle':
                if np.linalg.norm(position - entity.position) < (size + entity.size):
                    return True

            elif entity.shape == 'rectangle':
                if check_circle_rectangle_collision(entity.position, position, entity.size2, size):
                    return True

        elif shape == 'rectangle':
            if entity.shape == 'circle':
                if check_circle_rectangle_collision(position, entity.position, size, entity.size2):
                    return True
            else:
                rect1 = pygame.Rect(position[0] - size[0] / 2, position[1] - size[1] / 2, size[0], size[1])
                rect2 = pygame.Rect(entity.position[0] - entity.position[0] / 2, entity.position[1] - entity.position[1] / 2,
                                    entity.position[0], entity.size2[1])
                if rect1.colliderect(rect2):
                    return True

    return False


def check_circle_rectangle_collision(rect_position, circle_position, rect_size, circle_radius):
    circle_center = circle_position
    rect_center = rect_position
    rect_half_size = np.array(rect_size) / 2

    # Find the closest point to the circle within the rectangle
    closest_point = np.clip(circle_center, rect_center - rect_half_size, rect_center + rect_half_size)

    # Calculate the distance between the circle's center and this closest point
    distance = np.linalg.norm(circle_center - closest_point)

    return distance < circle_radius
