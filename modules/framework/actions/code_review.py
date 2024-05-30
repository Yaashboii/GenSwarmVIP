import asyncio
import time

from modules.framework.action import ActionNode
from modules.framework.response.code_parser import SingleFunctionParser
from modules.framework.code.function_node import FunctionNode, State
from modules.prompt.code_review_stage_prompt import HIGH_LEVEL_FUNCTION_REVIEW
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
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

    async def operate_on_node(self, function_node: FunctionNode):
        self._function = function_node
        return await self.run()


class CodeReviewAsync(ActionNode):
    def __init__(self, next_text="", node_name=""):
        super().__init__(next_text, node_name)

    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        function_pool = FunctionTree()

        async def operation(function):
            action = CodeReview()
            action.setup(function)
            return await action.run()

        layer_index = function_pool.get_min_layer_index_by_state(State.WRITTEN)
        if not all(
            function_node.state == State.WRITTEN
            for function_node in function_pool._layers[layer_index].functions
        ):
            logger.log("All functions in the layer are not in WRITTEN state", "error")
            # TODO: 解决当出现生成的函数跑到前面层的问题。跑到后面层是通过重置State来解决的，但是跑到前面层的问题还没有解决
            time.sleep(1)
            raise SystemExit
        await function_pool.process_function_layer(
            operation, operation_type=State.REVIEWED, start_layer_index=layer_index
        )
