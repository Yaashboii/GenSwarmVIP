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
