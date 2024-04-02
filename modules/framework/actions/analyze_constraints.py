import json

from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import ANALYZE_CONSTRAINT_PROMPT_TEMPLATE, CONSTRAIN_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code


class AnalyzeConstraints(ActionNode):
    def _build_prompt(self):
        user_constraints = {"constraints": []}
        for constraint in self._context.constraint_pool.constraints.values():
            user_constraints["constraints"].append(
                {
                    "name": constraint.name,
                    "description": constraint.description
                }
            )
        self.prompt = ANALYZE_CONSTRAINT_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self._context.user_command.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            output_template=CONSTRAIN_TEMPLATE,
            user_constraints=json.dumps(user_constraints, indent=4)
        )

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response, lang='json')
        self._context.constraint_pool.add_constraints(code)
        self._context.log.format_message(f"Analyze Constraints Success", "success")
        return response
    
    def _can_skip(self) -> bool:
        return False
    
if __name__ == '__main__':
    analyst = AnalyzeConstraints("contraints")
    from modules.utils import root_manager
    import asyncio
    # for i in range(30):
    path = '/src/tests'
    root_manager.update_root(path, set_data_path=False)
    analyst._context.user_command.message = (
        "Form a flock with other robots, navigating together by keeping aligned, spaced out, "
        "and cohesive. Avoid obstacles and stay away from the environment's edges and obstacles."
    )
    asyncio.run(analyst.run())
    analyst._context.save_to_file(f'{path}/analyze_stage.pkl')