import argparse
import pickle
from abc import ABC, abstractmethod

from modules.framework.files.file import File
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.context.function_info import FunctionPool

class Context(ABC):
    @abstractmethod
    def save_to_file(self, filename):
        pass
    
    @abstractmethod
    def load_from_file(self, filename):
        pass
    
    @property
    def command():
        raise NotImplementedError

class WorkflowContext(Context):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialize()

        return cls._instance

    def _initialize(self):
        self.user_command = File(name='command.md')
        self.function_pool = FunctionPool(name='functions.py')
        self.design_result = File(name='design_result.py')
        self.constraint_pool = ConstraintPool(name='constraints.md')
        self.function_list = []
        self.run_code = File(name='run.py', message="""import sys
from functions import run_loop
robot_id = sys.argv[1]
if __name__ == '__main__':
    run_loop()
""")
        # self.sequence_diagram = FileInfo(name='sequence_diagram.md')
        self.run_result = File(name='run_result.md')
        self.args = argparse.Namespace()

    def save_to_file(self, file_path):
        with open(file_path, 'wb') as file:
            pickle.dump(self._instance, file)

    def load_from_file(self, file_path):
        with open(file_path, 'rb') as file:
            self._instance = pickle.load(file)

    @property
    def command(self):
        return self._instance.user_command.message
    
    @command.setter
    def command(self, value):
        self._instance.user_command.message = value

    @property
    def constraints_value(self):
        return self._instance.constraint_pool.constraints.values()
    
    @property
    def constraints_dict(self):
        return self._instance.constraint_pool.constraints

    @property
    def constraints(self):
        return self._instance.constraint_pool.message
      
    @property
    def functions_value(self):
        self._instance.function_pool.functions.values()

    @property
    def function_layer(self):
        self._instance.function_pool.function_layer

    @property
    def parameters(self):
        return self._instance.parameters.message
    
    @parameters.setter
    def parameters(self, value):
        self._instance.parameters.message = value

    @property
    def function_content(self):
        return self._instance.function_pool.functions_content()

    def add_constraint(self, constraint):
        self._instance.constraint_pool.add_constraints(constraint)

    def init_functions(self, code):
        self._instance.function_pool.init_functions(code)

    def add_functions(self, content):
        self._instance.function_pool.add_functions(content)

    def check_function_grammar(self, function_name):
        self._instance.function_pool.check_function_grammar(function_name)

    def add_sat_func(self, constraint_name, function_name):
        self._instance.constraint_pool.add_sat_func(constraint_name, function_name)

    def update_message(self):
        self._instance.function_pool.update_message()

    def set_function_definition(self, function_name, definition):
        self._instance.function_pool.functions[function_name].definition = function


if __name__ == '__main__':
    context = WorkflowContext()
