from modules.file import logger
from modules.framework.action import ActionNode, AsyncNode
from modules.framework.code import State, FunctionNode
from modules.framework.code_error import Bug, Bugs
from modules.framework.parser import GrammarParser
from modules.utils import root_manager



class GrammarCheck(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self.function_name: str
        self.function: FunctionNode
        self._grammar_checker = GrammarParser()

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
            errors = self._grammar_checker.check_code_errors(
                file_path=f"{root_manager.workspace_root}/functions.py"
            )
            return self._process_response(str(errors))

    def _process_response(self, response: str) -> str | Bugs | Bug:
        if eval(response):
            bug_list = [
                Bug(error_msg=e["error_message"], error_function=e["function_name"])
                for e in eval(response)
            ]
            logger.log(
                f"Grammar check failed for function: {self.function_name}", "error"
            )
            return Bugs(bug_list)
        else:
            self.function.state = State.CHECKED
            logger.log(
                f"Grammar check passed for function: {self.function_name}", "warning"
            )
            return response


class GrammarCheckAsync(AsyncNode):
    def __init__(
            self, run_mode='layer', start_state=State.REVIEWED, end_state=State.CHECKED
    ):
        super().__init__(run_mode, start_state, end_state)

    def _build_prompt(self):
        pass

    async def operate(self, function: FunctionNode):
        action = GrammarCheck()
        action.error_handler = self.error_handler
        action.setup(function)
        return await action.run()

    async def _run_layer_mode(self):
        layer_index = self.function_pool.get_min_layer_index_by_state(self._start_state)
        if layer_index == -1:
            logger.log("No functions in NOT_STARTED state", "error")
            raise SystemExit

        if not all(
                function_node.state == self._start_state
                for function_node in self.function_pool._layers[layer_index].functions
        ):
            logger.log(
                "All functions in the layer are not in NOT_STARTED state", "error"
            )
            raise SystemExit

        for function_node in self.function_pool._layers[layer_index].functions:
            await self.operate(function_node)
            function_node.state = State.CHECKED

    async def _run_sequential_mode(self):
        for function in self.function_pool.nodes:
            if function.state == self._start_state:
                await self.operate(function)

    async def _run(self):
        if self._run_mode == "layer":
            await self._run_layer_mode()
        elif self._run_mode == "sequential":
            await self._run_sequential_mode()
            for function in self.function_pool.nodes:
                function.state = self._end_state
        elif self._run_mode == "parallel":
            await self._run_sequential_mode()
            for function in self.function_pool.nodes:
                function.state = self._end_state
        else:
            logger.log("Unknown generate_mode", "error")
            raise SystemExit
