import argparse
import pickle
from abc import ABC, abstractmethod

from modules.framework.context.file_info import FileInfo, logger
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.context.function_info import FunctionPool

class Context(ABC):
    @abstractmethod
    def foo():
        pass


class WorkflowContext(Context):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialize()

        return cls._instance

    def _initialize(self):
        self.user_command = FileInfo(name='command.md')
        self.function_pool = FunctionPool(name='functions.py')
        self.design_result = FileInfo(name='design_result.py')
        self.constraint_pool = ConstraintPool(name='constraints.md')
        self.function_list = []
        self.run_code = FileInfo(name='run.py', message="""import sys
from functions import run_loop
robot_id = sys.argv[1]
if __name__ == '__main__':
    run_loop()
""")
        # self.sequence_diagram = FileInfo(name='sequence_diagram.md')
        self.run_result = FileInfo(name='run_result.md')
        self.args = argparse.Namespace()

    @classmethod
    def save_to_file(cls, file_path):
        with open(file_path, 'wb') as file:
            pickle.dump(cls._instance, file)

    @classmethod
    def load_from_file(cls, file_path):
        with open(file_path, 'rb') as file:
            cls._instance = pickle.load(file)

