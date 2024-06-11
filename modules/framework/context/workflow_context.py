import argparse
import pickle
from abc import ABC, abstractmethod

from modules.file.file import File
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.code.function_tree import FunctionTree


class Context(ABC):
    @abstractmethod
    def save_to_file(self, filename):
        pass

    @abstractmethod
    def load_from_file(self, filename):
        pass

    @property
    def command(self):
        raise NotImplementedError


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
import sys
import threading
import rospy


def run_robot(robot_id):
    from functions import run_loop, initialize_ros_node
    initialize_ros_node(robot_id=robot_id)
    run_loop()


def run_multiple_robot(start_idx, end_idx):
    rospy.init_node(f'multi_robot_publisher_node{start_idx}_{end_idx}', anonymous=True)
    threads = []
    for i in range(start_idx, end_idx + 1):
        thread = threading.Thread(target=run_robot, args=(i,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    a = sys.argv
    start_id = int(sys.argv[1])
    end_id = int(sys.argv[2])
    run_multiple_robot(start_id, end_id)
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


if __name__ == "__main__":
    context = WorkflowContext()
