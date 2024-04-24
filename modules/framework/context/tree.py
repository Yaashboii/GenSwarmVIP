from modules.framework.context import FunctionNode
from modules.file.log_file import logger

class FunctionLayer:
    def __init__(self, function_set: set = None):
        if function_set is None:
            function_set = set()
        self._layer : set[FunctionNode] = function_set
        self._next : FunctionLayer = None
        self._index = 0

    @property
    def next(self):
        return self._next
    
    @property
    def functions(self):
        return list(self._layer)
    
    @property
    def set_callers(self):
        result = set()
        for function_node in self._layer:
            result |= function_node.callers
        return result
    
    @next.setter
    def next(self, value: 'FunctionLayer'):
        self._next = value

    def __len__(self):
        return len(self.functions)

    def __iter__(self):
        return self

    def __next__(self):
        if self._index >= len(self._layer):
            self._index = 0
            raise StopIteration
        value = self.functions[self._index]
        self._index += 1
        return value

    def add_function(self, function: FunctionNode):
        self._layer.add(function)

class FunctionTree:
    def __init__(self):
        self._function_nodes : dict[str, FunctionNode] = {}
        self._layers : list[FunctionLayer] = []
        self._layer_head : FunctionLayer =  None
        self._current_layer : FunctionLayer = None 
        self._index = 0
        self._keys_set = set()
            
    def __iter__(self):
        return self

    def __next__(self):
        if self._index >= len(self._layers):
            self._index = 0
            raise StopIteration
        value = self._layers[self._index]
        self._index += 1
        return value    
        # if not self._current_layer.next:
        #     self._current_layer = self._layer_head
        #     raise StopIteration
        # value = self._current_layer
        # self._current_layer = self._current_layer.next
        # return value

    def __getitem__(self, key):
        if isinstance(key, int):
            if key >= len(self._layers) or key < -len(self._layers):
                raise IndexError("Index out of range")
            return self._layers[key]
        elif isinstance(key, slice):
            start, stop, step = key.indices(len(self._layers))
            sliced_layers = [self._layers[i] for i in range(start, stop, step)]
            return sliced_layers 
        elif isinstance(key, str):
            if key not in self._function_nodes:
                return FunctionNode(name=key, description='')
            return self._function_nodes[key]
        else:
            raise TypeError("Invalid index type")
        
    def __setitem__(self, key, value):
        if isinstance(key, str):
            self._function_nodes[key] = value

    @property
    def nodes(self):
        return self._function_nodes.values()
    
    @property
    def keys(self):
        return self._function_nodes.keys()
    
    @property
    def keys_set(self):
        return self.keys_set or set(self.keys)
    
    def _reset(self):
        self._layers.clear()
        self._layer_head = None
    
    def update(self):
        self._reset()
        self._layer_head =  self._get_bottom_layer()
        self._current_layer = self._layer_head
        set_visited_nodes = set()
        self._build_up(self._layer_head, set_visited_nodes)
        logger.log(f"layers: {[[f.name for f in layer] for layer in self._layers]}", level='warning')
        

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