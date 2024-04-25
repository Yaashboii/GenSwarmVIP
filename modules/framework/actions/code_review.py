

import asyncio

from modules.framework.action import ActionNode
from modules.framework.response.code_parser import SingleFunctionParser
from modules.framework.code.function_node import FunctionNode
from modules.prompt.code_review_stage_prompt import HIGH_LEVEL_FUNCTION_REVIEW
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.framework.response.parser import parse_text
from modules.file.log_file import logger
from modules.framework.code.function_tree import FunctionTree


class CodeReview(ActionNode):
    def __init__(self, next_text='', node_name=''):
        super().__init__(next_text, node_name)
        self._function : FunctionNode = None
        self._function_pool = FunctionTree()

    def _build_prompt(self):
        other_functions : list[FunctionNode] = self._function_pool.filtered_functions(self._function)
        other_functions_str = '\n\n'.join([f.function_body for f in other_functions])
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

    def _process_response(self, response: str) -> str:
        try:
            desired_function_name = self._function._name
            code = parse_text(text=response)
            code_obj = SingleFunctionParser(code)
            code_obj.parse_code(code)
            code_obj.check_function_name(desired_function_name)
            code_obj.save_to_pool()
            self._function_pool.save_and_check_functions([desired_function_name])
            # # TODO,add bug fix mechanism for such cases,rather than just raising exception to trigger retry
            # if errors:
            #     logger.log(
            #         f"High Level Function Review Failed: Function {desired_function_name} has syntax error: {errors}",
            #         "error")
            #     raise Exception
            return code
        except ValueError as e:
            logger.log(f"No function detected in the response: {e}", 'warning')
        except Exception as e:
            logger.log(f"High Level Function Review Failed: {e}", "error")
            raise Exception  # trigger retry


class CodeReviewAsync(ActionNode):
    def __init__(self, next_text='', node_name=''):
        super().__init__(next_text, node_name)
        self._function_pool = FunctionTree()

    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        async def operation(function):
            action = CodeReview()
            action.setup(function)
            return await action.run()
        self._function_pool.process_function_layers(operation, start_layer_index=1)
        

if __name__ == "__main__":
    from modules.utils import root_manager

    path = '../../../workspace/test'
    root_manager.update_root(path)

    code_review = CodeReviewAsync("code review")
    code_review.context.load_from_file(path + "/write_run.pkl")
    asyncio.run(code_review.run())

    code_review.context.save_to_file(f'{path}/code_review.pkl')
