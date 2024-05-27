import time

from modules.framework.action import ActionNode
from modules.framework.code.function_node import State, FunctionNode
from modules.framework.code.function_tree import FunctionTree
from modules.framework.code.grammar_checker import GrammarChecker
from modules.framework.code_error import Bug, Bugs

from modules.file.log_file import logger
from modules.utils import root_manager


class GrammarCheck(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self.function_name: str
        self.function: FunctionNode
        self._grammar_checker = GrammarChecker()

    def _build_prompt(self):
        pass

    def setup(self, function: FunctionNode):
        self.function_name = function.name
        self.function = function

    async def _run(self) -> str:
        if not self.function_name:
            logger.log("Error in GrammarCheck: function_name is not set", "error")
            raise SystemExit
        else:
            self.context._function_pool.save_by_function(function=self.function_name)
            errors = self._grammar_checker.check_code_errors(file_path=f"{root_manager.workspace_root}/functions.py")
            return str(self._process_response(errors))

    def _process_response(self, response: str) -> str | Bugs | Bug:
        if response:
            bug_list = [
                Bug(error_msg=e["error_message"], error_function=e["function_name"])
                for e in eval(response)
            ]
            logger.log(f"Grammar check failed for function: {self.function_name}", "error")
            return Bugs(bug_list)
        else:
            self.function.state = State.CHECKED
            logger.log(f"Grammar check passed for function: {self.function_name}", "warning")
            return response


class GrammarCheckAsync(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)

    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        function_pool = FunctionTree()

        async def operation(function: FunctionNode):
            action = GrammarCheck()
            action.setup(function)
            return await action.run()

        layer_index = function_pool.get_min_layer_index_by_state(State.WRITTEN)
        if not all(function_node.state == State.WRITTEN for function_node in
                   function_pool._layers[layer_index].functions):
            logger.log("All functions in the layer are not in WRITTEN state", "error")
            # TODO: 解决当出现生成的函数跑到前面层的问题。跑到后面层是通过重置State来解决的，但是跑到前面层的问题还没有解决
            time.sleep(1)
            raise SystemExit
        # layer_index = function_pool.get_min_layer_index_by_state(State.REVIEWED)
        # if not all(function_node.state == State.REVIEWED for function_node in
        #            function_pool._layers[layer_index].functions):
        #     logger.log("All functions in the layer are not in REVIEWED state", "error")
        #     # TODO: 解决当出现生成的函数跑到前面层的问题。跑到后面层是通过重置State来解决的，但是跑到前面层的问题还没有解决
        #     time.sleep(1)
        #     raise SystemExit
        for function_node in function_pool._layers[layer_index].functions:
            await operation(function_node)
