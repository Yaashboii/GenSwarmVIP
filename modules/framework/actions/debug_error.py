import asyncio
import os
import sys

from modules.framework.action import ActionNode
from modules.utils import parse_code
from modules.prompt.run_code_prompt import DEBUG_PROMPT
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.task_description import TASK_DES


class DebugError(ActionNode):
    def __init__(self, next_text='', node_name=''):
        super().__init__(next_text, node_name)
        self.error = None

    def setup(self, error):
        self.error = error

    def _build_prompt(self):
        mentioned_function = ""
        for function in self.context.function_pool.functions.values():
            if function.name in self.error:
                mentioned_function += "\n\n" + function.content
        self.prompt = DEBUG_PROMPT.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            mentioned_functions=mentioned_function,
            error_message=self.error
        )

    def _process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        self._context.function_pool.add_functions(content=code)
        return str(code)
