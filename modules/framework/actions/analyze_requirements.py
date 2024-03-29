from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import ANALYZE_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES

class AnalyzeReqs(ActionNode):
    def __init__(self, next_text: str , node_name: str = ''):
        super().__init__(next_text, node_name)
        self.prompt = ANALYZE_PROMPT_TEMPLATE.format(
            instruction=self._context.user_command.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES
        )

    def _process_response(self, response: str) -> str:
        self._context.analysis.message = response
        self._context.log.format_message(f"Analyze Requirements Success", "success")
        return response
