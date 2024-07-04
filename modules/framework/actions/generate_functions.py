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
from modules.utils.function import CodeAnalyzer


class GenerateFunctions(ActionNode):

    def __init__(self, next_text: str = "", node_name: str = "", run_mode='layer'):
        super().__init__(next_text, node_name)
        self._cost_time = 0
        self._function_pool = FunctionTree()
        design_functions = DesignFunctionAsync(run_mode)
        write_functions = WriteFunctionsAsync(run_mode)
        grammar_check = GrammarCheckAsync(run_mode)

        code_review = CodeReviewAsync(run_mode)
        bug_handler = BugLevelHandler()
        write_run = WriteRun("code")

        debug_error = DebugError("debug error")
        bug_handler.next_action = debug_error
        debug_error._next = grammar_check
        grammar_check.error_handler = bug_handler

        # link actions
        self._actions = ActionLinkedList('Generate Functions', design_functions)
        self._actions.add(write_functions)
        self._actions.add(code_review)
        self._actions.add(grammar_check)

        self._write_run = ActionLinkedList('Write Run', write_run)
        self._write_run.add(code_review)
        self._write_run.add(grammar_check)

    def _build_prompt(self):
        pass

    def _process_response(self, response: str) -> str:
        pass

    async def _run(self) -> str:
        start_time = time.time()
        finish = False
        while not finish:
            time.sleep(1)
            await self._actions.run_internal_actions()
            finish = all(node.state == State.CHECKED for node in self._function_pool.nodes)
        await self._write_run.run_internal_actions()
        end_time = time.time()
        self._cost_time = end_time - start_time
        code_analyzer = CodeAnalyzer(code=self._function_pool.file.read())
        self._average_complexity, self._mi_score = code_analyzer.analyze()
        logger.log(f"Generate functions cost time: {self._cost_time}", "warning")
        logger.log(f"average complexity: {self._average_complexity}, mi score: {self._mi_score}", "warning")
        logger.log("All functions are generated", "warning")
