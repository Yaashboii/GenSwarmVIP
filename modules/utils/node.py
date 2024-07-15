from abc import ABC


class Node(ABC):
    def __init__(self, name, description):
        self._name = name
        self._description = description
        self._connections: set[Node] = set()

    @property
    def brief(self):
        return f"**{self._name}**: {self._description}"

    @property
    def connections(self):
        return self._connections

    @property
    def name(self):
        return self._name

    @property
    def description(self):
        return self._description

    def connect_to(self, node: "Node"):
        if node not in self._connections:
            self._connections.add(node)
            node.connect_to(self)

    def has_no_connections(self):
        return len(self._connections) == 0

