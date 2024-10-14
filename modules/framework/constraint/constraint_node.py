from modules.framework.node import Node


class ConstraintNode(Node):
    def __init__(self, name, description):
        super().__init__(name, description)

    def to_json(self):
        result = {"name": self._name, "description": self._description}
        return result
