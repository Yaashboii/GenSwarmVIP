from modules.framework.code.function_layer import FunctionLayer
from modules.framework.code.function_node import FunctionNode
from modules.framework.context.contraint_info import ConstraintPool
from modules.file.log_file import logger
from modules.file.file import File

class FunctionTree:
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._function_nodes : dict[str, FunctionNode] = {}
            cls.import_list: set[str] = {'from apis import *'}
            cls._layers : list[FunctionLayer] = []
            cls._layer_head : FunctionLayer =  None
            cls._current_layer : FunctionLayer = None 
            cls._index = 0
            cls._keys_set = set()
            cls._file = File(name='functions.py')
        return cls._instance

    def __getitem__(self, key: str):
        if key not in self._function_nodes:
            return FunctionNode(name=key, description='')
        return self._function_nodes[key]
        # else:
        #     raise TypeError("Invalid index type")
        
    def __setitem__(self, key, value):
        if isinstance(key, str):
            self._function_nodes[key] = value
    
    def reset(self):
        self._function_nodes : dict[str, FunctionNode] = {}
        self.import_list: set[str] = {'from apis import *'}
        self._layers : list[FunctionLayer] = []
        self._layer_head : FunctionLayer =  None
        self._current_layer : FunctionLayer = None 
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
        self._layer_head =  self._get_bottom_layer()
        self._current_layer = self._layer_head
        set_visited_nodes = set()
        self._build_up(self._layer_head, set_visited_nodes)
        logger.log(f"layers: {[[f.name for f in layer] for layer in self._layers]}", level='warning')

    def init_functions(self, content: str):
        constraint_pool = ConstraintPool()
        try:
            for function in eval(content)['functions']:
                name = function['name']
                self[name] = FunctionNode(name, function['description'])
                [self[name].connect_to(constraint_pool[constraint_name]) 
                 for constraint_name in function["constraints"]]
                [self[name].add_callee(self[call]) for call in function['calls']] 
            self.update()
        except Exception as e:
            logger.log(f'Error in init_functions: {e}', level='error')
            raise Exception
        
    async def process_function_layers(self, operation, start_layer_index=0, check_grammer=True):
        import asyncio
        for index, layer in enumerate(self._layers[start_layer_index:]):
            tasks = []
            logger.log(f"Layer: {start_layer_index+index}", "warning")
            for function_node in layer:
                task = asyncio.create_task(operation(function_node))
                tasks.append(task)
            await asyncio.gather(*tasks)
            # layer_index = current_layer_index if current_layer_index < len(
                # self._function_layer) else len(self._function_layer) - 1
            # current_layer = self._function_layer[layer_index]
        # if check_grammer: 
            # self._check_function_grammer_by_layer(layer)

    def _reset_layers(self):
        self._layers.clear()
        self._layer_head = None
    

    def _build_up(self, current_layer : FunctionLayer, set_visited_nodes: set):
        if len(current_layer) > 0:
            self._layers.append(current_layer)
        else:
            return
        next_layer = FunctionLayer()

        for caller in current_layer.set_callers - set_visited_nodes:
            set_visited_nodes.add(caller)
            next_layer.add_function(self._function_nodes[caller.name])
        self._build_up(next_layer, set_visited_nodes)

    def _get_bottom_layer(self):
        bottom_layer = [
            func for func in self._function_nodes.values()
            if func.callees.isdisjoint(set(self._function_nodes.values()))
        ]
        return FunctionLayer(bottom_layer)
    
    def set_definiton(self, function_name, definition):
        self._function_nodes[function_name]._definition = definition
    
    # def _check_function_grammer_by_layer(self, current_layer):
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

        print(list(seen))

        if function not in seen and f_name in self._function_nodes:
            seen.add(function)
            callees = self._function_nodes[f_name].callees
            [self._find_all_relative_functions(callee, seen) for callee in callees]
            
        return list(seen)
    
    def _save_functions_to_file(self, functions: list[FunctionNode] = None):
        import_str = "\n".join(sorted(self.import_list))
        content = '\n\n\n'.join([f.content for f in functions])
        self._file.message = f"{import_str}\n\n{content}\n"

    def _save_by_function(self, function: FunctionNode):
        relative_function = self._find_all_relative_functions(function)
        logger.log(f"relative_function: {relative_function}", level='warning')
        self._save_functions_to_file(relative_function)

    def _save_imports(self, imports):
        self.import_list |= imports

    def _save_function_dict(self, function_dict: dict[str, str]):
        for name, content in function_dict.items():
            self._function_nodes[name].content = content
            # self._function_tree[name].add_import(self._imports)
            for other_function in self._function_nodes.values():
                if (other_function._name != name and other_function._name in content):
                    self._function_nodes[name].add_callee(other_function)
            # logger.log(f" function_name: {name}, "
            #            f"calls: {self._function_tree[name].callees}", 
            #            level='info')
        self.update()