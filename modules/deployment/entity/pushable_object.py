from .base_entity import Entity


class PushableObject(Entity):
    def __init__(self, object_id, initial_position, size, color='yellow'):
        super().__init__(object_id,
                         initial_position,
                         size,
                         color=color,
                         collision=True,
                         moveable=True)
        self.__target_position = None

    @property
    def target_position(self):
        """Get the target position of the robot."""
        return self.__target_position

    @target_position.setter
    def target_position(self, value):
        """Set a new target position for the robot."""
        self.__target_position = value
