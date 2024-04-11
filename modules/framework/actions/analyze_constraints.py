import json

from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import ANALYZE_CONSTRAINT_PROMPT_TEMPLATE, CONSTRAIN_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code
from modules.framework.context import logger

class AnalyzeConstraints(ActionNode):
    def _build_prompt(self):
        user_constraints = {"constraints": []}
        for constraint in self.context.constraints_value:
            user_constraints["constraints"].append(
                {
                    "name": constraint.name,
                    "description": constraint.description
                }
            )
        self.prompt = ANALYZE_CONSTRAINT_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self.context.command,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            output_template=CONSTRAIN_TEMPLATE,
            user_constraints=json.dumps(user_constraints, indent=4)
        )

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response, lang='json')
        self.context.add_constraint(code)
        logger.log(f"Analyze Constraints Success", "success")
        return response

if __name__ == '__main__':
    analyst = AnalyzeConstraints("constraints")
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/test'
    root_manager.update_root(path, set_data_path=False)
    analyst.context.command = (
        "Form a flock with other robots, navigating together by keeping aligned, spaced out, "
        "and cohesive. Avoid obstacles and stay away from the environment's edges and obstacles."
    )
    asyncio.run(analyst.run())
    analyst.context.save_to_file(f'{path}/analyze_constraints.pkl')
