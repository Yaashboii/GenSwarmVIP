from .entity import Entity


class Landmark(Entity):
    def __init__(self, landmark_id, initial_position, size, color):
        super().__init__(landmark_id,
                         initial_position,
                         size,
                         color=color,
                         collision=False,
                         moveable=False)
