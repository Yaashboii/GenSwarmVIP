from modules.file.file import File, logger
from modules.framework.code.function_node import FunctionNode
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.code.function_tree import FunctionTree
from modules.framework.code.grammer_checker import GrammarChecker

class FunctionPool():
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls.import_list: set[str] = {'from apis import *'}
            cls._function_tree = FunctionTree()
            cls._file = File(name='functions.py')
            cls._grammar_checker = GrammarChecker()
        return cls._instance
    
    @property
    def functions_body(self):
        result = [f.function_body for f in self._function_tree.nodes]
        return result
    
    @property
    def functions_brief(self):
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
        result = list(filter(lambda f: f.name in content == 0, self.functions_body))
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
        
    def update_function_tree(self, function_dict: dict[str, str]):
        for name, content in function_dict.items():
            self._function_tree[name].content = content
            # self._function_tree[name].add_import(self._imports)
            for other_function in self._function_tree.nodes:
                if (other_function._name != name and other_function._name in content):
                    self._function_tree[name].add_callee(other_function)
            logger.log(f" function_name: {name}, "
                       f"calls: {self._function_tree[name].callees}", 
                       level='info')
        self._function_tree.update()
    
    def save_and_check_functions(self, function_list: list):
        for function_name in function_list:
            self._check_function_grammar(function_name)
            self._check_caller_function_grammer(function_name)
    
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
        

    def _save_by_function(self, function: FunctionNode):
        relative_function = self._find_all_relative_functions(function)
        logger.log(f"relative_function: {relative_function}", level='warning')
        self._save_functions_to_file(relative_function)

    def _check_function_grammar(self, function_name):
        self._save_by_function(self._function_tree[function_name])

        errors = self._grammar_checker.check_code_errors(self._file.file_path)
        if errors:
            logger.log(f'Grammar check failed for {function_name}', level='error')
        else:
            logger.log(f'Grammar check passed for {function_name}', level='debug')
        return errors
    
    def _check_caller_function_grammer(self, function_name):
        [self._check_function_grammar(f.name) 
         for f in self._function_tree[function_name].callers]

    def _find_all_relative_functions(self, function: FunctionNode, seen: set = None):
        if seen is None:
            seen = set()     
        f_name = function.name   

        if function not in seen and function in self._function_tree.nodes:
            seen.add(function)
            calls = self._function_tree[f_name].callees
            [self._find_all_relative_functions(call, seen) for call in calls]
            
        return list(seen)

    def _save_functions_to_file(self, functions: list[FunctionNode] = None):
        # def combine_unique_imports(import_list):
        #     unique_imports = set()
        #     for import_str in import_list:
        #         for import_line in import_str.splitlines():
        #             unique_imports.add(import_line.strip())

        #     combined_imports = "\n".join(sorted(unique_imports))
        #     return combined_imports
        
        import_str = "\n".join(sorted(self.import_list))
        content = '\n\n\n'.join([f.content for f in functions])
        self._file.message = f"{import_str}\n\n{content}\n"

    def _check_function_grammer_by_layer(self, current_layer):
        try:
            errors = []
            for function in current_layer:
                error = self._check_function_grammar(function)
                errors.append(error)
        except Exception as e:
            import traceback
            logger.log(f"error occurred in grammar check:\n {traceback.format_exc()}", 'error')
            raise SystemExit(f"error occurred in async write functions{e}")
