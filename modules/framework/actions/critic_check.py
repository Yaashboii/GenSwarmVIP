import asyncio

from modules.framework.action import ActionNode
from modules.prompt.data_critic_stage_prompt import FILTER_CONSTRAINTS_TEMPLATE, OUTPUT_FORMAT
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.task_description import TASK_DES
from modules.framework.code.code import parse_text
from modules.framework.context import ConstraintPool

class CriticCheck(ActionNode):
    def _build_prompt(self):
        constraint_pool = ConstraintPool()
        self.prompt = FILTER_CONSTRAINTS_TEMPLATE.format(
            task_des=TASK_DES,
            data_api=ROBOT_API,
            output_format=OUTPUT_FORMAT,
            constraints=str(constraint_pool),
        )

    def _process_response(self, response: str) -> str:
        code = parse_text(text=response, lang='json')
        return code