import argparse
import pickle
from enum import Enum

from collections import defaultdict


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
        self.name = name
        self._message = message
        self.root = root
        self.status = FileStatus.NOT_WRITTEN
        self.version = 0

    @property
    def message(self):
        if not self._message:
            from modules.utils import root_manager, read_file
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
        from modules.utils import root_manager, write_file
        self.root = root_manager.workspace_root
        write_file(self.root, self.name, content)


class FileLog(FileInfo):
    def __init__(self, name: str = '', message: str = '', root: str = ''):
        super().__init__(name, message, root)
        from modules.utils import setup_logger, LoggerLevel
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)

    @property
    def message(self):
        return super().message

    @message.setter
    def message(self, content: str):
        from modules.utils import root_manager
        self.root = root_manager.workspace_root
        from modules.utils import write_file
        write_file(self.root, self.name, content, mode='a')

    def log(self, content: str, level: str = 'info'):
        """
        Formats a message based on the provided style and logs the content.

        :param content: The message content to be formatted and logged.
        :param level: The style to format the message. Supported styles: stage, action,
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
            'info': '#### <span style="color: black;">info: </span>\n{}\n',
            'debug': '#### <span style="color: black;">debug: </span>\n{}\n',
        }

        # Verify level is supported
        if level not in color_mapping:
            self._logger.error(f"Level {level} is not supported")
        log_action = {
            'stage': self._logger.info,
            'action': self._logger.debug,
            'prompt': self._logger.debug,
            'response': self._logger.info,
            'success': self._logger.info,
            'error': self._logger.error,
            'warning': self._logger.warning,
            'info': self._logger.info,
            'debug': self._logger.debug,
        }.get(level, self._logger.info)

        log_action(content)

        self.message = color_mapping[level].format(content)


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
        self.definition = None
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
            logger.log(f'Error in init_functions: {e}', level='error')
            raise Exception

    def add_functions(self, content: str):
        from modules.utils import extract_imports_and_functions, extract_top_level_function_names
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
            self.functions[function_name].calls = []
            for other_function in self.functions.values():
                if other_function.name != function_name and other_function.name in function_content[0]:
                    self.functions[function_name].calls.append(other_function.name)
            logger.log(f" function_name: {function_name}, calls: {self.functions[function_name].calls}", level='info')

        self.function_layer = self.build_layers_from_bottom()

    def check_function_grammar(self, function_name: str):
        from modules.utils import check_grammar, find_function_name_from_error
        relative_function = self.extend_calls(function_name)
        logger.log(f"relative_function: {relative_function}", level='warning')
        self.update_message(relative_function)

        errors = check_grammar(str(self.root / self.name))
        if errors:
            for e in errors:
                error_function_name = find_function_name_from_error(file_path=str(self.root / self.name),
                                                                    error_line=e['line'])
                logger.log(f"{error_function_name}: {e['error_message']}", level='error')
            logger.log(f'Grammar check failed for {function_name}', level='error')
        else:
            logger.log(f'Grammar check passed for {function_name}', level='debug')
        return errors

    def extend_calls(self, function_name: str, seen: set = None):
        if seen is None:
            seen = set()

        if function_name not in seen:
            if function_name in self.functions.keys():
                seen.add(function_name)
                calls = self.functions[function_name].calls
                for call in calls:
                    if call not in seen:
                        self.extend_calls(call, seen)
        return list(seen)

    def update_message(self, function_name: str | list[str] = None):
        from modules.utils import combine_unique_imports
        import_str = combine_unique_imports(self.import_list)
        self.message = f"{import_str}\n\n{self.functions_content(function_name)}\n"

    def functions_content(self, function_name: str | list[str] = None):
        if function_name:
            if isinstance(function_name, str):
                function_name = [function_name]
            return '\n\n\n'.join([self.functions[f].content for f in function_name])
        return '\n\n\n'.join([f.content for f in self.functions.values()])

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
        logger.log(f"layers: {[[f.name for f in layer] for layer in layers]}", level='warning')
        return layers


class ConstraintPool(FileInfo):
    def __init__(self, name: str = '', root: str = ''):
        super().__init__(name=name, root=root)
        self.constraints: dict[str, ConstraintInfo] = {}

    def add_constraints(self, content: str):
        try:
            for constraint in eval(content)['constraints']:
                name = constraint['name']
                self.constraints[name] = ConstraintInfo(
                    name=name,
                    description=constraint['description']
                )
        except Exception as e:
            logger.log(f'Error in add_constraints: {e}', level='error')
            raise Exception
        self.update_message()

    def update_message(self):
        self.message = self.constraints_content()

    def add_sat_func(self, constraint_name, function_name: str | list[str]):
        if isinstance(function_name, str):
            function_name = [function_name]
        if constraint_name in self.constraints:
            self.constraints[constraint_name].satisfyingFuncs.extend(function_name)
        else:
            logger.log(f"Constraint {constraint_name} not found,Current Existing:{self.constraints.values()}",
                       level='error')
            raise SystemExit

    def constraints_content(self):
        return '\n'.join([c.text for c in self.constraints.values()])


class WorkflowContext:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialize()

        return cls._instance

    def _initialize(self):
        self.logger = logger
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


logger = FileLog(name='log.md')
