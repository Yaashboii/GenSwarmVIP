import argparse
import pickle

from modules.file import File
from modules.framework.code import FunctionTree
from modules.framework.constraint import ConstraintPool

from .context import Context


class WorkflowContext(Context):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialize()

        return cls._instance

    def _initialize(self):
        self.user_command = File(name="command.md")
        self.design_result = File(name="design_result.py")
        self._parameters: File = File(name="parameters.md")
        self.feedbacks = []
        self.run_code = File(
            name="run.py",
            message="""
""",
        )
        # self.sequence_diagram = FileInfo(name='sequence_diagram.md')
        self.run_result = File(name="run_result.md")
        self.args = argparse.Namespace()
        self._constraint_pool = ConstraintPool()
        self._function_pool = FunctionTree()

    def save_to_file(self, file_path):
        with open(file_path, "wb") as file:
            pickle.dump(self._instance, file)

    def load_from_file(self, file_path):
        with open(file_path, "rb") as file:
            self._instance = pickle.load(file)

    def set_root_for_files(self, root_value):
        for file_attr in vars(self).values():
            if isinstance(file_attr, File):
                file_attr.root = root_value
            if isinstance(file_attr, FunctionTree):
                file_attr.file.root = root_value

    @property
    def command(self):
        return self._instance.user_command.message

    @command.setter
    def command(self, value):
        self._instance.user_command.message = value

    @property
    def parameters(self):
        return self._instance.parameters.message

    @parameters.setter
    def parameters(self, value):
        self._instance.parameters.message = value
