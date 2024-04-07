from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import PARAMETER_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code


class AnalyzeFunctions(ActionNode):
    def _build_prompt(self):
        self.prompt = PARAMETER_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            function_des='\n'.join([f.text for f in self._context.function_pool.functions.values()]),
            env_des=ENV_DES,
            robot_api=ROBOT_API,
        )

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response)
        self._context.parameters.message = code
        self._context.logger.logger(f"Design Parameters success!", "success")
        return response

    def _can_skip(self) -> bool:
        return False
