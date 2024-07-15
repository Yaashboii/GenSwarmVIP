from modules.utils import Node
from enum import Enum


class State(Enum):
    NOT_STARTED = 0
    DESIGNED = 1
    WRITTEN = 2
    REVIEWED = 3
    CHECKED = 4


class FunctionNode(Node):
    def __init__(self, name, description):
        super().__init__(name, description)
        self._import_list: set[str] = set()
        self._callees: set[FunctionNode] = set()
        self._callers: set[FunctionNode] = set()
        self.content: str = ""
        self._definition: str = ""
        self._state: State = State.NOT_STARTED
        self._num_of_lines: int = 0


    @property
    def callees(self):
        return self._callees

    @property
    def callee_names(self):
        return [c.name for c in self._callees]

    @property
    def callers(self):
        return self._callers

    @property
    def description(self):
        return self._description

    @description.setter
    def description(self, value):
        self._description = value

    @property
    def body(self):
        return self.content or self._definition

    @body.setter
    def body(self, value):
        self.content = value

    @property
    def function_body(self):
        return "\n".join(list(self._import_list) + [self.content])

    def add_import(self, import_content: set):
        self._import_list |= import_content

    def add_callee(self, function: "FunctionNode"):
        if function not in self._callees:
            self._callees.add(function)
            function.add_caller(self)

    def add_caller(self, function: "FunctionNode"):
        if function not in self._callers:
            self._callers.add(function)
            function.add_callee(self)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        if isinstance(value, int) and value in range(len(State)):
            self._state = State(value)
        elif isinstance(value, State):
            self._state = value
        else:
            raise ValueError(
                "Invalid state. Must be an integer or an instance of FunctionNode.State."
            )

    def reset(self):
        self._import_list.clear()
        self.content = ""
        self._definition = ""
        self._state = State.NOT_STARTED
