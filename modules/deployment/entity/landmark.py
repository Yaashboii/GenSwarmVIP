from .base_entity import Entity


class Landmark(Entity):
    def __init__(self, landmark_id, initial_position, size, color, shape='rectangle'):
        super().__init__(landmark_id, initial_position, size, color=color, collision=False, movable=False,
                         shape=shape)
