from modules.framework.code.function_layer import FunctionLayer
from modules.framework.code.function_node import FunctionNode
from modules.framework.context.contraint_info import ConstraintPool
from modules.file.log_file import logger
from modules.file.file import File
from modules.framework.error import GrammarError
from modules.framework.code.grammer_checker import GrammarChecker


class FunctionTree:
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance.reset()
            cls._grammar_checker = GrammarChecker()
        return cls._instance

    def __getitem__(self, key: str):
        return self._function_nodes[key]

    def __setitem__(self, key, value):
        if isinstance(key, str):
            self._function_nodes[key] = value

    def reset(self):
        self._function_nodes: dict[str, FunctionNode] = {}
        self.import_list: set[str] = {'from apis import *'}
        self._layers: list[FunctionLayer] = []
        self._index = 0
        self._keys_set = set()
        self._file = File(name='functions.py')

    @property
    def nodes(self):
        return self._function_nodes.values()

    @property
    def names(self):
        return self._function_nodes.keys()

    @property
    def keys_set(self):
        return self._keys_set or set(self.names)

    @property
    def functions_body(self):
        result = [f.function_body for f in self._function_nodes.values()]
        return result

    @property
    def functions_brief(self):
        result = [f.brief for f in self._function_nodes.values()]
        return result

    @property
    def function_valid_content(self):
        result = [f.content for f in self._function_nodes.values() if f.content]
        return result

    def filtered_functions(self, exclude_function: FunctionNode):
        result = [value for key, value in self._function_nodes.items()
                  if key != exclude_function.name]
        return result

    def related_function_content(self, error_msg: str):
        result = [value.content for key, value in self._function_nodes.items()
                  if key in error_msg]
        return result

    def update(self):
        self._reset_layers()
        layer_head = self._get_bottom_layer()
        set_visited_nodes = set()
        self._build_up(layer_head, set_visited_nodes)
        logger.log(f"layers: {[[f.name for f in layer] for layer in self._layers]}", level='warning')

    def init_functions(self, content: str):
        constraint_pool = ConstraintPool()
        try:
            for function in eval(content)['functions']:
                name = function['name']
                new_node = self._obtain_node(name, description=function['description'])
                [new_node.connect_to(constraint_pool[constraint_name])
                 for constraint_name in function["constraints"]]
                from modules.prompt.robot_api_prompt import robot_api
                [new_node.add_callee(self._obtain_node(name=call))
                 for call in function['calls'] if call not in robot_api.apis.keys()]
            self.update()
        except Exception as e:
            logger.log(f'Error in init_functions: {e}', level='error')
            raise Exception

    async def process_function_layers(self, operation, start_layer_index=0, check_grammar=True):
        import asyncio
        for index, layer in enumerate(self._layers[start_layer_index:]):
            tasks = []
            logger.log(f"Layer: {start_layer_index + index}", "warning")
            for function_node in layer:
                task = asyncio.create_task(operation(function_node))
                tasks.append(task)
            await asyncio.gather(*tasks)
            # layer_index = current_layer_index if current_layer_index < len(
            # self._function_layer) else len(self._function_layer) - 1
            # current_layer = self._function_layer[layer_index]
        # if check_grammar:
        # self._check_function_grammar_by_layer(layer)

    def _reset_layers(self):
        self._layers.clear()

    def _build_up(self, current_layer: FunctionLayer, set_visited_nodes: set):
        if len(current_layer) > 0:
            self._layers.append(current_layer)
        else:
            return
        next_layer = FunctionLayer()

        for caller in current_layer.set_callers - set_visited_nodes:
            set_visited_nodes.add(caller)
            next_layer.add_function(self[caller.name])
        self._build_up(next_layer, set_visited_nodes)

    def _get_bottom_layer(self):
        bottom_layer = [
            func for func in self._function_nodes.values()
            if func.callees.isdisjoint(set(self._function_nodes.values()))
        ]
        return FunctionLayer(bottom_layer)

    def set_definition(self, function_name, definition):
        self._function_nodes[function_name]._definition = definition

    def update_from_parser(self, imports: set, function_dict: dict):
        self._update_imports(imports)
        self._update_function_dict(function_dict)
        # logger.log(self.functions_body)

        self.update()

    def save_code(self, function_names):
        for function_name in function_names:
            self._save_by_function(self._function_nodes[function_name])

    def check_grammar(self, function_names):
        for function_name in function_names:
            self._check_function_grammar(function_name)
            self._check_caller_function_grammar(function_name)

    def _check_function_grammar(self, function_name):
        errors = self._grammar_checker.check_code_errors(self._file.file_path)
        status = 'passed' if errors else 'failed'
        raise GrammarError(message=f'Grammar check {status} for {function_name}',
                           grammar_error=errors)

    def _check_caller_function_grammar(self, function_name):
        [self._check_function_grammar(f.name)
         for f in self._function_nodes[function_name].callers]

    # def _check_function_grammar_by_layer(self, current_layer):
    #     try:
    #         errors = []
    #         for function in current_layer:
    #             error = self._check_function_grammar(function)
    #             errors.append(error)
    #     except Exception as e:
    #         import traceback
    #         logger.log(f"error occurred in grammar check:\n {traceback.format_exc()}", 'error')
    #         raise SystemExit(f"error occurred in async write functions{e}")

    def _find_all_relative_functions(self, function: FunctionNode, seen: set = None):
        if seen is None:
            seen = set()
        f_name = function.name

        if function not in seen and f_name in self._function_nodes:
            seen.add(function)
            callees = self._function_nodes[f_name].callees
            [self._find_all_relative_functions(callee, seen) for callee in callees]

        return list(seen)

    def save_functions_to_file(self, functions: list[FunctionNode] = None):
        import_str = "\n".join(sorted(self.import_list))
        if not functions:
            content = '\n\n\n'.join([f.content for f in self._function_nodes.values()])
        else:
            content = '\n\n\n'.join([f.content for f in functions])
        self._file.message = f"{import_str}\n\n{content}\n"

    def _save_by_function(self, function: FunctionNode):
        relative_function = self._find_all_relative_functions(function)
        logger.log(f"relative_ function: {relative_function}", level='warning')
        self.save_functions_to_file(relative_function)

    def _update_imports(self, imports: set):
        self.import_list |= imports

    def _update_function_dict(self, function_dict: dict[str, str]):
        for name, content in function_dict.items():
            node = self._obtain_node(name, content=content)
            # self._function_tree[name].add_import(self._imports)
            for other_function in self._function_nodes.values():
                if (other_function._name != name and other_function._name in content):
                    node.add_callee(other_function)
            logger.log(f" function_name: {name}, calls: {self[name].callee_names}",
                       level='info')

    def _obtain_node(self, name, description='', content=''):
        if name in self._function_nodes:
            node = self._function_nodes[name]
        else:
            node = FunctionNode(name=name, description=description)
            self._function_nodes[name] = node
        node.content = content or node.content
        node.description = description or node.description

        return node
