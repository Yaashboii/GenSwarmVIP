from modules.framework.context import FunctionNode
from modules.file.log_file import logger

class FunctionLayer:
    def __init__(self, function_set=()):
        self._layer : set[FunctionNode] = function_set
        self._next : FunctionLayer = None
        self._index = 0

    @property
    def next(self):
        return self._next
    
    @property
    def functions(self):
        return list(self._layer)
    
    @next.setter
    def next(self, value: 'FunctionLayer'):
        self._next = value

    def __len__(self):
        return len(self._layer)

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
        self._visited_nodes = set()
        self._layers : list[FunctionLayer] = []
        self._layer_head : FunctionLayer =  None
        self._current_layer : FunctionLayer = None 
        self._index = 0
            
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
        else:
            raise TypeError("Invalid index type")
    
    def update(self, function_dict):
        self._function_nodes = function_dict
        self._layer_head =  self._get_bottom_layer()
        self._current_layer = self._layer_head
        self._build_up(self._layer_head)
        logger.log(f"layers: {[[f.name for f in layer] for layer in self._layers]}", level='warning')
        

    def _build_up(self, current_layer : FunctionLayer):
        next_layer = FunctionLayer()
        for function_node in current_layer:
            for caller in function_node.callers:
                if caller not in self._visited_nodes:
                    self._visited_nodes.add(caller)
                    next_layer.add_function(self._function_nodes[caller.name])
        if len(next_layer) > 0:
            self._layers.append(next_layer)
            self._build_up(next_layer)
        current_layer = next_layer

    def _get_bottom_layer(self):
        bottom_layer = [
            func for func in self._function_nodes.values()
            if func.callees.isdisjoint(set(self._function_nodes.values()))
        ]
        return FunctionLayer(bottom_layer)