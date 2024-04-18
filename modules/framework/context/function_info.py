from collections import defaultdict
from modules.file.file import File, logger
from modules.framework.code.code import Code  
from modules.framework.context.contraint_info import ConstraintPool

# TODO: function pool and constraint pool should be rewrite totally
class _FunctionInfo:
    def __init__(self, description, name):
        self._name = name
        self._description = description
        self._import_list = []
        self._calls = []
        self._satisfying_constraints: list[str] = []
        self._content = None
        self._definition = None

    @property
    def function_info(self):
        return f"**{self._name}**: {self._description}"
    
    @property
    def calls(self):
        return self._calls
    
    @calls.setter
    def calls(self, value):
        self._calls = value
    
    def add_import(self, import_content):
        self._import_list.extend(import_content)

    def add_call(self, function_name):
        self._calls.append(function_name)


class FunctionPool(File):
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__()
            cls._import_list: list[str] = ['from apis import *']
            cls._functions: dict[str, _FunctionInfo] = {}
            cls._function_layer: list[_FunctionInfo] = []
            cls._file = File(name='functions.py')
        return cls._instance

    def init_functions(self, content: str):
        try:
            for function in eval(content)['functions']:
                name = function['name']
                self._functions[name] = _FunctionInfo(
                    name=name,
                    description=function['description'],
                )
                self._functions[name]._satisfying_constraints = function['constraints']
                self._functions[name].calls = function['calls']

            self._function_layer = self.build_layers_from_bottom()
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
                _FunctionInfo(name=function_name, description='')
            )._content = function_content[0]
            self._functions[function_name].add_import(import_content)
            self._functions[function_name].calls = []
            for other_function in self._functions.values():
                if other_function._name != function_name and other_function._name in function_content[0]:
                    self._functions[function_name].add_call(other_function._name)
            logger.log(f" function_name: {function_name}, calls: {self._functions[function_name].calls}", level='info')

        self._function_layer = self.build_layers_from_bottom()

    def check_function_grammar(self, function_name: str):
        from modules.utils import check_grammar, find_function_name_from_error
        relative_function = self.extend_calls(function_name)
        logger.log(f"relative_function: {relative_function}", level='warning')
        self.update_message(relative_function)

        errors = check_grammar(str(self._root / self._name))
        if errors:
            for e in errors:
                error_function_name = find_function_name_from_error(
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
        from modules.utils import combine_unique_imports
        import_str = combine_unique_imports(self._import_list)
        self.message = f"{import_str}\n\n{self.functions_content(function_name)}\n"

    def functions_content(self, function_name: str | list[str] = None):
        if function_name:
            if isinstance(function_name, str):
                function_name = [function_name]
            return '\n\n\n'.join([self._functions[f]._content for f in function_name])
        return '\n\n\n'.join([f._content for f in self._functions.values()])

    def build_layers_from_bottom(self):
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
    
    def sync_constraints_to(self, constriant_pool: ConstraintPool):
        for function in self._functions.values():
            for constraint in function._satisfying_constraints:
                constriant_pool.add_satisfying_func(constraint_name=constraint, function_name=function._name)


