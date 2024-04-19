from collections import defaultdict
import subprocess
import re

from modules.file.file import File, logger
from modules.framework.code.code import Code  
from modules.framework.context.node import FunctionNode
from modules.framework.context.contraint_info import ConstraintPool


class FunctionPool(File):
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__()
            cls._import_list: list[str] = ['from apis import *']
            cls._functions: dict[str, FunctionNode] = {}
            cls._function_layer: list[FunctionNode] = []
            cls._file = File(name='functions.py')
        return cls._instance
    
    @property
    def function_contents(self):
        result = [f.function_body for f in self._functions.values()]
        return result
    
    @property
    def function_infos(self):
        result = [f.brief for f in self._functions.values()]
        return result
    
    @property
    def function_valid_content(self):
        result = [f._content for f in self._functions.values() if f._content]
        return result
    
    def filtered_function_content(self, exclude_function: FunctionNode):
        result = list(filter(lambda f: f._name != exclude_function._name == 0, self.function_contents))
        return result
    
    def filtered_function_definition(self, exclude_function: FunctionNode):
        definitons = [f._content if f._content else f._definition for f in self._functions.values()]
        result = list(filter(lambda f: f._name != exclude_function._name == 0, definitons))
        return result
    
    def filtered_function_info(self, exclude_function: FunctionNode):
        result = list(filter(lambda f: f._name != exclude_function._name == 0, self.function_infos))
        return result
    
    def related_function_content(self, content):
        result = list(filter(lambda f: f._name in content == 0, self.function_contents))
        return result

    def init_functions(self, content: str):
        constraint_pool = ConstraintPool()
        try:
            for function in eval(content)['functions']:
                name = function['name']
                self._functions[name] = FunctionNode(
                    name=name,
                    description=function['description'],
                )

                [self._functions[name].connect_to(constraint_pool[constraint_name]) 
                 for constraint_name in function["constraints"]]
                
                self._functions[name].calls = function['calls']

            self._function_layer = self._build_layers_from_bottom()
        except Exception as e:
            logger.log(f'Error in init_functions: {e}', level='error')
            raise Exception

    def add_functions(self, content: str):
        code = Code(content)
        import_list, function_list = code.extract_imports_and_functions(content)
        self._import_list.extend(import_list)
        for function in function_list:
            code_obj = Code(function)
            function_name = code_obj.extract_top_level_function_names()[0]
            import_content, function_content = code.extract_imports_and_functions(function)
            self._functions.setdefault(
                function_name,
                FunctionNode(name=function_name, description='')
            )._content = function_content[0]
            self._functions[function_name].add_import(import_content)
            self._functions[function_name].calls = []
            for other_function in self._functions.values():
                if other_function._name != function_name and other_function._name in function_content[0]:
                    self._functions[function_name].add_reference_function(other_function)
            logger.log(f" function_name: {function_name}, calls: {self._functions[function_name].calls}", level='info')

        self._function_layer = self._build_layers_from_bottom()

    def check_function_grammar(self, function):
        function_name = function._name
        relative_function = self.extend_calls(function_name)
        logger.log(f"relative_function: {relative_function}", level='warning')
        self.update_message(relative_function)

        errors = self._check_grammar(str(self._root / self._name))
        if errors:
            for e in errors:
                error_function_name = self._find_function_name_from_error(
                    file_path=str(self._root / self._name),
                    error_line=e['line']
                )
                logger.log(f"{error_function_name}: {e['error_message']}", level='error')
            logger.log(f'Grammar check failed for {function_name}', level='error')
        else:
            logger.log(f'Grammar check passed for {function_name}', level='debug')
        return errors

    def extend_calls(self, function_name: str, seen: set = None):
        if seen is None:
            seen = set()

        if function_name not in seen:
            if function_name in self._functions.keys():
                seen.add(function_name)
                calls = self._functions[function_name].calls
                for call in calls:
                    if call not in seen:
                        self.extend_calls(call, seen)
        return list(seen)

    def update_message(self, function_name: str | list[str] = None):
        import_str = self._combine_unique_imports(self._import_list)
        self.message = f"{import_str}\n\n{self.functions_content(function_name)}\n"

    def functions_content(self, function_name: str | list[str] = None):
        if function_name:
            if isinstance(function_name, str):
                function_name = [function_name]
            return '\n\n\n'.join([self._functions[f]._content for f in function_name])
        return '\n\n\n'.join([f._content for f in self._functions.values()])

    def _build_layers_from_bottom(self):
        bottom_layer_functions = [
            func for func in self._functions.values()
            if all(called_func not in self._functions for called_func in func.calls)
        ]

        callers_of = defaultdict(list)
        for func_name, func_info in self._functions.items():
            for called_func in func_info.calls:
                if called_func in self._functions:
                    callers_of[called_func].append(func_name)

        layers = [bottom_layer_functions]
        visited = set(bottom_layer_functions)
        current_layer = bottom_layer_functions

        while current_layer:
            next_layer = []
            for func in current_layer:
                for caller in callers_of[func._name]:
                    if caller not in visited:
                        visited.add(caller)
                        next_layer.append(self._functions[caller])
            if next_layer:
                layers.append(next_layer)
            current_layer = next_layer
        logger.log(f"layers: {[[f._name for f in layer] for layer in layers]}", level='warning')
        return layers

    def check_caller_function_grammer(self, function_name):
        [self.check_function_grammar(f._name) 
         for f in self._functions.values() 
         if function_name in f._reference_functions]
        
    def _check_grammar(self, file_path: str):
        command = [
            'pylint',
            # '--disable=W,C,I,R --enable=E,W0612',
            '--disable=W,C,I,R ',
            file_path
        ]

        try:
            process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            result = process.stdout + process.stderr

            pattern = re.compile(r"(.*?):(\d+):(\d+): (\w+): (.*) \((.*)\)")
            matches = pattern.findall(result)

            errors = []
            for match in matches:
                file_path, line, column, error_code, error_message, _ = match
                errors.append({
                    "file_path": file_path,
                    "line": int(line),
                    "column": int(column),
                    "error_code": error_code,
                    "error_message": error_message
                })

            return errors
        except Exception as e:
            logger.log(f"Error occurred when check grammar: {e}", level='error')
            raise Exception(f"Error occurred when check grammar:{e}")

    def _find_function_name_from_error(self, file_path, error_line):
        with open(file_path, 'r') as file:
            lines = file.readlines()
            error_code_line = lines[error_line - 1].strip()
            for i in range(error_line - 2, -1, -1):
                if lines[i].strip().startswith('def '):
                    function_name = lines[i].strip().split('(')[0].replace('def ', '')
                    return function_name, error_code_line
        return None, error_code_line
    
    def _combine_unique_imports(self, import_list):
        unique_imports = set()

        for import_str in import_list:
            import_lines = import_str.splitlines()
            for import_line in import_lines:
                unique_imports.add(import_line.strip())

        combined_imports = "\n".join(sorted(unique_imports))

        return combined_imports
    
    def set_definiton(self, function_name, definition):
        self._functions[function_name]._definition = definition


    def async_handle_function_by_layer(self, operation, start_layer_index=0, check_grammer=True):
        import asyncio
        current_layer_index = start_layer_index
        while current_layer_index < len(self._function_layer):
            tasks = []
            current_layer = self._function_layer[current_layer_index]
            logger.log(f"Layer: {current_layer_index}", "warning")
            for function in current_layer:
                task = asyncio.create_task(operation(function))
                tasks.append(task)
            asyncio.gather(*tasks)

            layer_index = current_layer_index if current_layer_index < len(
                self._function_layer) else len(self._function_layer) - 1
            current_layer = self._function_layer[layer_index]
            if check_grammer: self._check_function_grammer_by_layer(current_layer)
            current_layer_index += 1

    def _check_function_grammer_by_layer(self, current_layer):
        try:
            errors = []
            for function in current_layer:
                error = self.check_function_grammar(function)
                errors.append(error)
        except Exception as e:
            import traceback
            logger.log(f"error occurred in grammar check:\n {traceback.format_exc()}", 'error')
            raise SystemExit(f"error occurred in async write functions{e}")

    

