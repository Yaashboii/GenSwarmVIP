from abc import ABC, abstractmethod

class Node(ABC):
    def __init__(self, name, description):
        self._name = name
        self._description = description
        self._connections : list[Node] = set()

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

    def connect_to(self, node: 'Node'):
        if node not in self._connections:
            self._connections.add(node)
            node.connect_to(self)

    def has_no_connections(self):
        return len(self._connections) == 0


class ConstraintNode(Node):
    def __init__(self, name, description):
        super().__init__(name, description)

    def to_json(self):
        result = {
            "name": self._name,
            "description": self._description
        }
        return result

class FunctionNode(Node):
    def __init__(self, name, description):
        super().__init__(name, description)
        self._import_list : list[str] = []
        self._callees : set[FunctionNode] = []
        self._callers : set[FunctionNode] = []
        self.content = None
        self._definition = None

    @property
    def callees(self):
        return self._callees
    
    @property
    def callers(self):
        return self._callers

    @property
    def function_body(self):
        return '\n'.join(self._import_list) + self.content

    def add_import(self, import_content):
        self._import_list.extend(import_content)

    def add_callee(self, function : 'FunctionNode'):
        if function not in self._callees:
            self._callees.add(function)
            function.add_caller(self)

    def add_caller(self, function : 'FunctionNode'):
        if function not in self._callers:
            self._callers.add(function)
            function.add_callee(self)


    