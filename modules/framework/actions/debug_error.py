from modules.framework.action import ActionNode
from modules.framework.code.code import parse_code
from modules.prompt.run_code_prompt import DEBUG_PROMPT
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.task_description import TASK_DES
from modules.framework.context import FunctionPool
from modules.framework.code.code import AstParser

class DebugError(ActionNode):
    def __init__(self, next_text='', node_name=''):
        super().__init__(next_text, node_name)
        self.error = None
        self._function_pool = FunctionPool()

    def setup(self, error):
        self.error = error

    def _build_prompt(self):
        mentioned_function = "\n\n".join(self._function_pool.related_function_content(self.error))
        self.prompt = DEBUG_PROMPT.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            mentioned_functions=mentioned_function,
            error_message=self.error
        )

    def _process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        code_obj = AstParser(code)
        code_obj.parse(code)
        code_obj.save_to_pool()
        return str(code)
