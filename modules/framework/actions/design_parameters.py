from modules.framework.action import ActionNode
from modules.prompt import (
    PARAMETER_PROMPT_TEMPLATE,
    ROBOT_API,
    ENV_DES,
    TASK_DES,
)
from modules.framework.response.text_parser import parse_text
from modules.file.log_file import logger
from modules.framework.code.function_tree import FunctionTree


class AnalyzeFunctions(ActionNode):
    def _build_prompt(self):
        function_pool = FunctionTree()
        self.prompt = PARAMETER_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            function_des="\n".join(function_pool.functions_brief),
            env_des=ENV_DES,
            robot_api=ROBOT_API,
        )

    def _process_response(self, response: str) -> str:
        code = parse_text(text=response)
        self.context.parameters = code
        logger.log(f"Design Parameters success!", "success")
        return response
