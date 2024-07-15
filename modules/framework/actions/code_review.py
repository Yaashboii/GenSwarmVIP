import asyncio
import time

from modules.framework.action import ActionNode, AsyncNode
from modules.framework.response.code_parser import SingleFunctionParser
from modules.framework.code.function_node import FunctionNode, State
from modules.prompt import  (
    HIGH_LEVEL_FUNCTION_REVIEW,
    ROBOT_API,
    ENV_DES,
    TASK_DES,
)
from modules.framework.response.text_parser import parse_text
from modules.file.log_file import logger
from modules.framework.code.function_tree import FunctionTree


class CodeReview(ActionNode):
    def __init__(self, next_text="", node_name=""):
        super().__init__(next_text, node_name)
        self._function: FunctionNode = None
        self._function_pool = FunctionTree()

    def _build_prompt(self):
        other_functions: list[FunctionNode] = self._function_pool.filtered_functions(
            self._function
        )
        other_functions_str = "\n\n".join([f.function_body for f in other_functions])
        self.prompt = HIGH_LEVEL_FUNCTION_REVIEW.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            function_name=self._function._name,
            other_functions=other_functions_str,
            function_content=self._function.content,
        )

    def setup(self, function: FunctionNode):
        self._function = function
        logger.log(f"Reviewing function: {self._function._name}", "warning")

    async def _process_response(self, response: str) -> str:
        try:
            desired_function_name = self._function._name
            code = parse_text(text=response)
            parser = SingleFunctionParser()
            parser.parse_code(code)
            parser.check_function_name(desired_function_name)
            self._function_pool.update_from_parser(parser.imports, parser.function_dict)
            self._function_pool.save_code([desired_function_name])
            # # TODO,add bug fix mechanism for such cases,rather than just raising exception to trigger retry
            # if errors:
            #     logger.log(
            #         f"High Level Function Review Failed: Function {desired_function_name} has syntax error: {errors}",
            #         "error")
            #     raise Exception
            return code
        except ValueError as e:
            logger.log(f"No function detected in the response: {e}", "warning")
        except Exception as e:
            logger.log(f"High Level Function Review Failed: {e}", "error")
            raise Exception  # trigger retry


class CodeReviewAsync(AsyncNode):
    def __init__(self, run_mode='layer', start_state=State.WRITTEN, end_state=State.REVIEWED):
        super().__init__(run_mode, start_state, end_state)

    def _build_prompt(self):
        pass

    async def operate(self, function):
        action = CodeReview()
        action.setup(function)
        return await action.run()


if __name__ == '__main__':
    import asyncio
    from modules.framework.context import WorkflowContext
    import argparse

    context = WorkflowContext()
    path = "../../../workspace/test"

    code_reviewer = CodeReviewAsync('sequential')
    code_reviewer.context.load_from_file(f"{path}/designed_function.pkl")
    asyncio.run(code_reviewer.run())
    context.save_to_file("../../../workspace/test/reviewed_function.pkl")
