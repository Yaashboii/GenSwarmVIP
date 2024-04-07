import asyncio

from modules.framework.action import ActionNode
from modules.prompt.data_critic_stage_prompt import FILTER_CONSTRAINTS_TEMPLATE, OUTPUT_FORMAT
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code


class CriticCheck(ActionNode):
    def _build_prompt(self):
        self.prompt = FILTER_CONSTRAINTS_TEMPLATE.format(
            task_des=TASK_DES,
            data_api=ROBOT_API,
            output_format=OUTPUT_FORMAT,
            constraints='\n'.join([c.text for c in self._context.constraint_pool.constraints.values()]),
        )

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response, lang='json')
        return code

    def _can_skip(self) -> bool:
        # TODO: can skip when files concerning to this critic are not changed
        return False
