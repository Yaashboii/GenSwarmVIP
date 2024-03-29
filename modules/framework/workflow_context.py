import argparse
import pickle
from enum import Enum

from collections import defaultdict, deque
from modules.utils import read_file, write_file
from modules.utils import setup_logger, LoggerLevel, format_log_message
from modules.utils import extract_top_level_function_names, extract_imports_and_functions, combine_unique_imports


class FileStatus(Enum):
    NOT_WRITTEN = 0
    NOT_TESTED = 1
    TESTED_FAIL = 2
    TESTED_PASS = 3


class RunResult(Enum):
    WITH_ERROR = 1
    WITHOUT_ERROR = 2


class FileInfo:
    def __init__(self, name: str = '', message: str = '', root: str = ''):
        super().__init__()
        self.name = name
        self._message = message
        self.root = root
        self.status = FileStatus.NOT_WRITTEN
        self.version = 0

    @property
    def message(self):
        if not self._message:
            from modules.utils import root_manager
            self.root = root_manager.workspace_root
            try:
                self._message = read_file(self.root, self.name)
            except FileNotFoundError:
                self._message = ''
        return self._message

    @message.setter
    def message(self, content: str):
        self._message = content
        if self.status == FileStatus.NOT_WRITTEN:
            self.status = FileStatus.NOT_TESTED
        from modules.utils import root_manager
        self.root = root_manager.workspace_root
        write_file(self.root, self.name, content)


# TODO: function pool and constraint pool should be rewrite totally
class FunctionInfo:
    def __init__(self, description, name):
        self.name = name
        self.description = description
        self.import_list = []
        self.parameters = []
        self.calls = []
        self.satisfying_constraints: list[str] = []
        self.content = None
        self.text = f"**{self.name}**: {self.description}"


class ConstraintInfo:
    def __init__(self, description, name):
        self.satisfyingFuncs: list[int] = []
        self.name = name
        self.description = description
        self.text = f"**{self.name}**: {self.description}"


class FunctionPool(FileInfo):

    def __init__(self, name: str = '', root: str = ''):
        super().__init__(name=name, root=root)
        self.import_list: list[str] = ['from apis import *']
        self.functions: dict[str, FunctionInfo] = {}
        self.parameters: dict = {}
        self.function_layer: list[FunctionInfo] = []

    def init_functions(self, content: str):
        try:
            for function in eval(content)['functions']:
                name = function['name']
                self.functions[name] = FunctionInfo(
                    name=name,
                    description=function['description'],
                )
                self.functions[name].satisfying_constraints = function['constraints']
                self.functions[name].calls = function['calls']

            self.function_layer = self.build_layers_from_bottom()
        except Exception as e:
            print('Error in init_functions: ', e)
            raise e

    def add_functions(self, content: str):
        import_list, function_list = extract_imports_and_functions(content)
        self.import_list.extend(import_list)
        for function in function_list:
            function_name = extract_top_level_function_names(function)[0]
            import_content, function_content = extract_imports_and_functions(function)
            self.functions.setdefault(
                function_name,
                FunctionInfo(name=function_name, description='')
            ).content = function_content[0]
            self.functions[function_name].import_list.extend(import_content)
        self.function_layer = self.build_layers_from_bottom()
        self.update_message()

    def update_message(self):
        import_str = combine_unique_imports(self.import_list)
        self.message = f"{import_str}\n\n{self.functions_content()}"

    def functions_content(self):
        return '\n\n'.join([f.content for f in self.functions.values() if f.content])

    def build_layers_from_bottom(self):
        bottom_layer_functions = [
            func for func in self.functions.values()
            if all(called_func not in self.functions for called_func in func.calls)
        ]

        callers_of = defaultdict(list)
        for func_name, func_info in self.functions.items():
            for called_func in func_info.calls:
                if called_func in self.functions:
                    callers_of[called_func].append(func_name)

        layers = [bottom_layer_functions]
        visited = set(bottom_layer_functions)
        current_layer = bottom_layer_functions

        while current_layer:
            next_layer = []
            for func in current_layer:
                for caller in callers_of[func.name]:
                    if caller not in visited:
                        visited.add(caller)
                        next_layer.append(self.functions[caller])
            if next_layer:
                layers.append(next_layer)
            current_layer = next_layer

        return layers


class ConstraintPool(FileInfo):
    def __init__(self, name: str = '', root: str = ''):
        super().__init__(name=name, root=root)
        self.constraints = {}

    def add_constraints(self, content: str):
        try:
            for constraint in eval(content)['constraints']:
                name = constraint['name']
                self.constraints[name] = ConstraintInfo(
                    name=name,
                    description=constraint['description']
                )
        except Exception as e:
            print('Error in add_constraints: ', e)
            raise e
        self.update_message()

    def update_message(self):
        self.message = self.constraints_content()

    def add_sat_func(self, constraint_name, function_name: str | list[str]):
        if isinstance(function_name, str):
            function_name = [function_name]
        if constraint_name in self.constraints:
            self.constraints[constraint_name].satisfyingFuncs.extend(function_name)
        else:
            raise SystemExit(f"Constraint {constraint_name} not found,Current Existing:{self.constraints.values()}")

    def constraints_content(self):
        return '\n'.join([c.text for c in self.constraints.values()])


class FileLog(FileInfo):
    def __init__(self, name: str = '', message: str = '', root: str = ''):
        super().__init__(name, message, root)
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)

    @property
    def message(self):
        return super().message

    @message.setter
    def message(self, content: str):
        from modules.utils import root_manager
        self.root = root_manager.workspace_root
        write_file(self.root, self.name, content, mode='a')

    def format_message(self, content: str, style: str):
        """
        Formats a message based on the provided style and logs the content.

        :param content: The message content to be formatted and logged.
        :param style: The style to format the message. Supported styles: stage, action,
                      prompt, response, success, error, warning.
        """
        color_mapping = {
            'stage': '***\n# <span style="color: blue;">Current Stage: *{}*</span>\n',
            'action': '## <span style="color: purple;">Current Action: *{}*</span>\n',
            'prompt': '### <span style="color: grey ;">Prompt: </span>\n{}\n',
            'response': '### <span style="color: black;">Response: </span>\n{}\n',
            'success': '#### <span style="color: gold;">Success: {}</span>\n',
            'error': '#### <span style="color: red;">Error: </span>\n{}\n',
            'warning': '#### <span style="color: orange;">Warning: </span>\n{}\n',
        }

        # Verify style is supported
        if style not in color_mapping:
            self._logger.error(f"Style {style} is not supported")
        log_action = {
            'stage': self._logger.info,
            'action': self._logger.debug,
            'prompt': self._logger.debug,
            'response': self._logger.info,
            'success': self._logger.info,
            'error': self._logger.error,
            'warning': self._logger.warning,
        }.get(style, self._logger.info)

        log_action(content)

        self.message = color_mapping[style].format(content)


class WorkflowContext:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialize()

        return cls._instance

    def _initialize(self):
        self.log = FileLog(name='log.md')
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


if __name__ == "__main__":
    # context = WorkflowContext()
    # context.log.message = ('Hello World!', 'prompt')
    log = FileLog(name='log.md')
    log.format_message('a', 'prompt')
