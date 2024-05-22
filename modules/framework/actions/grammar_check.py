from modules.framework.action import ActionNode
from modules.framework.code.grammar_checker import GrammarChecker
from modules.framework.code_error import Bug, Bugs

from modules.file.log_file import logger


class GrammarCheck(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self.function_name = None
        self._grammar_checker = GrammarChecker()

    def _build_prompt(self):
        pass

    def setup(self, function_name: str):
        self.function_name = function_name

    async def run(self) -> str:
        if not self.function_name:
            logger.log("Error in GrammarCheck: function_name is not set", "error")
            raise SystemExit
        else:
            self.context._function_pool.save_by_function(function=self.function_name)
            errors = self._grammar_checker.check_code_errors(file_path="function.py")
            return errors

    def _process_response(self, response: str) -> str | Bugs | Bug:
        if response:
            bug_list = [
                Bug(error_msg=e["error_message"], error_function=e["function_name"])
                for e in eval(response)
            ]
            return Bugs(bug_list)
        else:
            return response


class RunCodeAsync(ActionNode):
    async def _run(self):
        pass

    def _process_response(self, result: list):
        pass
