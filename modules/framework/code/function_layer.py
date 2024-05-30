import asyncio

from modules.framework.code.function_node import FunctionNode


class FunctionLayer:
    def __init__(self, function_set: set = None):
        if function_set is None:
            function_set = set()
        self._layer: set[FunctionNode] = function_set
        self._next: FunctionLayer = None
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
    def next(self, value: "FunctionLayer"):
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

    async def operate_on_nodes(self, operation):
        tasks = []
        # logger.log(f"Layer: {start_layer_index + index}", "warning")
        for function_node in self._layer:
            task = asyncio.create_task(operation(function_node))
            tasks.append(task)
        await asyncio.gather(*tasks)
