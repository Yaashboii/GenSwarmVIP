from modules.framework.action import ActionLinkedList, BaseNode
from modules.framework.code.function_tree import FunctionTree


class ActionOnLayer(ActionLinkedList):
    def __init__(self, head: BaseNode, name: str = "ActionOnLayer"):
        super().__init__(name=name, head=head)
        self._function_pool = FunctionTree()
        self._layer_iter = iter(self._function_pool)

    async def run(self, **kwargs):
        import asyncio

        action = self._head
        try:
            layer = next(self._layer_iter)
            while action:
                tasks = []
                for node in layer:
                    task = asyncio.create_task(action.operate_on_node(node))
                    tasks.append(task)
                await asyncio.gather(*tasks)

                action = action._next
        except StopIteration:
            self._function_pool.save_functions_to_file()
            return
