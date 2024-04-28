from modules.framework.context.node import Node

class FunctionNode(Node):
    def __init__(self, name, description):
        super().__init__(name, description)
        self._import_list : set[str] = set()
        self._callees : set[FunctionNode] = set()
        self._callers : set[FunctionNode] = set()
        self.content : str = ''
        self._definition: str = ''

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
        return '\n'.join(list(self._import_list) + [self.content])

    def add_import(self, import_content: set):
        self._import_list |= import_content

    def add_callee(self, function : 'FunctionNode'):
        if function not in self._callees:
            self._callees.add(function)
            function.add_caller(self)

    def add_caller(self, function : 'FunctionNode'):
        if function not in self._callers:
            self._callers.add(function)
            function.add_callee(self)