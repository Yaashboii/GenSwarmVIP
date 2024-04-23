from modules.file.file import File, logger
from modules.framework.code.code import AstParser  
from modules.framework.context.node import FunctionNode
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.context.tree import FunctionTree
from modules.framework.code.grammer_checker import GrammarChecker

class FunctionPool():
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__()
            cls._import_list: list[str] = ['from apis import *']
            cls._function_tree = FunctionTree()
            cls._file = File(name='functions.py')
            cls._grammar_checker = GrammarChecker()
        return cls._instance
    
    @property
    def function_contents(self):
        result = [f.function_body for f in self._function_tree.nodes]
        return result
    
    @property
    def function_infos(self):
        result = [f.brief for f in self._function_tree.nodes]
        return result
    
    @property
    def function_valid_content(self):
        result = [f.content for f in self._function_tree.nodes if f.content]
        return result
    
    def filtered_functions(self, exclude_function: FunctionNode):
        result = [value for value in self._function_tree.nodes
                  if value != exclude_function.name]
        return result
    
    def related_function_content(self, content):
        result = list(filter(lambda f: f.name in content == 0, self.function_contents))
        return result

    def init_functions(self, content: str):
        constraint_pool = ConstraintPool()
        try:
            for function in eval(content)['functions']:
                name = function['name']
                self._function_tree[name] = FunctionNode(
                    name=name,
                    description=function['description'],
                )

                [self._function_tree[name].connect_to(constraint_pool[constraint_name]) 
                 for constraint_name in function["constraints"]]
                
                [self._function_tree[name].add_callee(call) for call in function['calls']] 

            self._function_tree.update()
        except Exception as e:
            logger.log(f'Error in init_functions: {e}', level='error')
            raise Exception

    def add_functions(self, content: str):
        code = AstParser(content)
        self._import_list.extend(code.imports)
        for function in code.function_contents:
            code_obj = AstParser(function)
            function_name = code_obj.function_names()[0]
            self._function_tree[function_name].content = code_obj.function_contents[0]

            self._function_tree[function_name].add_import(code_obj.imports)
            self._function_tree[function_name].callees = []
            for other_function in self._function_tree.nodes:
                if (other_function._name != function_name and 
                    other_function._name in code_obj.function_contents[0]):
                    self._function_tree[function_name].add_callee(other_function)
            logger.log(f" function_name: {function_name}, "
                       f"calls: {self._function_tree[function_name].callees}", 
                       level='info')

        self._function_tree.update()

    def check_function_grammar(self, function: FunctionNode):
        function_name = function.name
        relative_function = self.extend_calls(function_name)
        logger.log(f"relative_function: {relative_function}", level='warning')
        self.update_message(relative_function)

        errors = self._grammar_checker.check_code_errors(self._file.file_path)
        if errors:
            logger.log(f'Grammar check failed for {function_name}', level='error')
        else:
            logger.log(f'Grammar check passed for {function_name}', level='debug')
        return errors

    def extend_calls(self, function_name: str, seen: set = None):
        if seen is None:
            seen = set()
        if function_name not in seen and function_name in self._function_tree.keys():
            seen.add(function_name)
            calls = self._function_tree[function_name].callees
            [self.extend_calls(call, seen) for call in calls if call not in seen]
        return list(seen)

    def update_message(self, function_name: str | list[str] = None):
        def combine_unique_imports(import_list):
            unique_imports = set()
            for import_str in import_list:
                for import_line in import_str.splitlines():
                    unique_imports.add(import_line.strip())

            combined_imports = "\n".join(sorted(unique_imports))
            return combined_imports
        
        import_str = combine_unique_imports(self._import_list)
        self._file.message = f"{import_str}\n\n{self.functions_content(function_name)}\n"

    def functions_content(self, function_name: str | list[str] = None):
        if function_name:
            if isinstance(function_name, str):
                function_name = [function_name]
            return '\n\n\n'.join([self._function_tree[f].content for f in function_name])
        return '\n\n\n'.join([f.content for f in self._function_tree.nodes])

    def check_caller_function_grammer(self, function_name):
        [self.check_function_grammar(f._name) 
         for f in self._function_tree.nodes
         if function_name in f._callees]
    
    def set_definiton(self, function_name, definition):
        self._function_tree[function_name]._definition = definition

    def process_function_layers(self, operation, start_layer_index=0, check_grammer=True):
        import asyncio
        for index, layer in enumerate(self._function_tree[start_layer_index:]):
            tasks = []
            logger.log(f"Layer: {start_layer_index+index}", "warning")
            for function_node in layer:
                task = asyncio.create_task(operation(function_node))
                tasks.append(task)
            asyncio.gather(*tasks)
            # layer_index = current_layer_index if current_layer_index < len(
                # self._function_layer) else len(self._function_layer) - 1
            # current_layer = self._function_layer[layer_index]
            if check_grammer: 
                self._check_function_grammer_by_layer(layer)

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
