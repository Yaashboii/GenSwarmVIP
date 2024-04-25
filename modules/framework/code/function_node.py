from modules.framework.context.node import Node

class FunctionNode(Node):
    def __init__(self, name, description):
        super().__init__(name, description)
        self._import_list : list[str] = []
        self._callees : set[FunctionNode] = set()
        self._callers : set[FunctionNode] = set()
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
        return '\n'.join(self._import_list + [self.content])

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