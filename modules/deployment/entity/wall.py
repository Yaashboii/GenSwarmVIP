from .entity import Entity


class Wall(Entity):
    def __init__(self, wall_id, initial_position, size: tuple):
        super().__init__(wall_id,
                         initial_position,
                         size,
                         color="gray",
                         collision=True,
                         moveable=False)
