from .entity import Entity


class Obstacle(Entity):
    def __init__(self, obstacle_id, initial_position, size):
        super().__init__(obstacle_id,
                         initial_position,
                         size,
                         color="gray",
                         collision=True,
                         moveable=False)
