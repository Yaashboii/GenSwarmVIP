from .base_entity import Entity


class Leader(Entity):
    def __init__(self, leader_id, initial_position, size):
        super().__init__(leader_id,
                         initial_position,
                         size,
                         color='red',
                         collision=True,
                         moveable=True)
