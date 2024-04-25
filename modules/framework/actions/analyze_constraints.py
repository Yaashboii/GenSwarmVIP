import json

from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import ANALYZE_CONSTRAINT_PROMPT_TEMPLATE, CONSTRAIN_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.framework.response.text_parser import parse_text
from modules.file.log_file import logger
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.response import *


class AnalyzeConstraints(ActionNode):
    def __init__(self, next_text, node_name = ''):
        super().__init__(next_text, node_name)
        self._constraint_pool : ConstraintPool = ConstraintPool()
        
    
    def _build_prompt(self):
        # constraints predefined
        user_constraints = {
            "constraints": self._constraint_pool.constaint_list
        }
        self.prompt = ANALYZE_CONSTRAINT_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self.context.command,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            output_template=CONSTRAIN_TEMPLATE,
            user_constraints=json.dumps(user_constraints, indent=4)
        )

    def _process_response(self, response: str) -> str:
        content = parse_text(response, 'json')
        self._constraint_pool.init_constraints(content)
        logger.log(f"Analyze Constraints Success", "success")

if __name__ == '__main__':
    analyst = AnalyzeConstraints("constraints")
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/test'
    root_manager.update_root(path)
    analyst.context.command = (
        "Form a flock with other robots, navigating together by keeping aligned, spaced out, "
        "and cohesive. Avoid obstacles and stay away from the environment's edges and obstacles."
    )
    asyncio.run(analyst.run())
    analyst.context.save_to_file(f'{path}/analyze_constraints.pkl')
