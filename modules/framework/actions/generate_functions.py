import argparse
import time

from modules.framework.action import ActionNode, ActionLinkedList
from modules.framework.actions import DesignFunctionAsync, WriteFunctionsAsync, CodeReviewAsync, WriteRun, DebugError
from modules.framework.actions.grammar_check import GrammarCheck, GrammarCheckAsync
from modules.framework.code.function_node import State
from modules.framework.handler import BugLevelHandler, FeedbackHandler
from modules.framework.response.text_parser import parse_text
from modules.file.log_file import logger
from modules.framework.code.function_tree import FunctionTree
from modules.utils import root_manager


class GenerateFunctions(ActionNode):

    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self._function_pool = FunctionTree()
        design_functions = DesignFunctionAsync("function definition")
        write_functions = WriteFunctionsAsync("function.py")
        grammar_check = GrammarCheckAsync("grammar check")
        code_review = CodeReviewAsync("reviewed code")
        bug_handler = BugLevelHandler()
        write_run = WriteRun("code")

        debug_error = DebugError("debug error")
        bug_handler.next_action = debug_error
        debug_error._next = grammar_check
        grammar_check.error_handler = bug_handler

        # link actions
        self._actions = ActionLinkedList('Generate Functions', design_functions)
        self._actions.add(write_functions)
        # self._actions.add(code_review)
        self._actions.add(grammar_check)

        self._write_run = ActionLinkedList('Write Run', write_run)
        # self._write_run.add(code_review)
        self._write_run.add(grammar_check)

    def _build_prompt(self):
        pass

    def _process_response(self, response: str) -> str:
        pass

    async def _run(self) -> str:
        finish = False
        while not finish:
            time.sleep(1)
            await self._actions.run_internal_actions()
            finish = all(node.state == State.CHECKED for node in self._function_pool.nodes)
        await self._write_run.run_internal_actions()
        logger.log("All functions are generated", "warning")


if __name__ == '__main__':
    import asyncio

    function_generator = GenerateFunctions("analyze constraints")

    path = "../../../workspace/test"
    parser = argparse.ArgumentParser(
        description="Run simulation with custom parameters."
    )

    parser.add_argument(
        "--print_to_terminal", type=bool, default=False, help="Whether to print the prompt and response to the terminal"
    )
    root_manager.update_root(path)
    args = parser.parse_args()
    function_generator.context.load_from_file(f"{path}/analyze_functions.pkl")
    function_generator.context.args = args

    asyncio.run(function_generator.run())
    function_generator.context.save_to_file(f"{path}/generate_functions.pkl")
