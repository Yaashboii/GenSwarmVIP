import asyncio
import json

from modules.stages.stage import Stage, StageResult
from modules.actions import AnalyzeFunctions, AnalyzeConstraints, DesignParameters
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.analyze_stage_prompt import ANALYZE_FUNCTION_PROMPT_TEMPLATE, ANALYZE_CONSTRAINT_PROMPT_TEMPLATE, \
    CONSTRAIN_TEMPLATE, FUNCTION_TEMPLATE, CLASS_DIAGRAM_PROMPT_TEMPLATE, PARAMETER_PROMPT_TEMPLATE
from modules.prompt.task_description import TASK_DES


class AnalysisStage(Stage):
    def __init__(self, action: AnalyzeConstraints):
        super(AnalysisStage, self).__init__()
        self._user_command = ""
        self._prompt = None
        self._action = action

    async def _analyze_functions(self):
        self._prompt = ANALYZE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self._context.user_command.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            constraints=self._context.constraint_pool.message,
            output_template=FUNCTION_TEMPLATE
        )
        self._action = AnalyzeFunctions()
        await self._action.run(prompt=self._prompt)

    # async def _analyze_class_diagram(self):
    #     self._prompt = CLASS_DIAGRAM_PROMPT_TEMPLATE.format(
    #         task_des=TASK_DES,
    #         instruction=self._context.user_command.message,
    #         robot_api=ROBOT_API,
    #         env_des=ENV_DES,
    #         constraints=self._context.constraint_pool.message,
    #     )
    #     self._action = AnalyzeConstraints()
    #     await self._action.run(prompt=self._prompt)

    async def _analyze_parameters(self):
        self._action = DesignParameters()
        self._prompt = PARAMETER_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            function_des='\n'.join([f.text for f in self._context.function_pool.functions.values()]),
            env_des=ENV_DES,
            robot_api=ROBOT_API,
        )
        await self._action.run(prompt=self._prompt)

    async def _analyze_constraints(self):
        user_constraints = {"constraints": []}
        for constraint in self._context.constraint_pool.constraints.values():
            user_constraints["constraints"].append(
                {
                    "name": constraint.name,
                    "description": constraint.description
                }
            )
        self._prompt = ANALYZE_CONSTRAINT_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self._context.user_command.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            output_template=CONSTRAIN_TEMPLATE,
            user_constraints=json.dumps(user_constraints, indent=4)
        )
        self._action = AnalyzeConstraints()
        await self._action.run(prompt=self._prompt)

    async def _run(self) -> StageResult:
        # await self._analyze_class_diagram()
        await self._analyze_constraints()
        await self._analyze_functions()
        # await self._analyze_parameters()
        return StageResult(keys=[])


if __name__ == '__main__':
    analyst = AnalysisStage(AnalyzeConstraints())
    from modules.utils import root_manager

    # for i in range(30):
    path = '/home/derrick/catkin_ws/src/code_llm/workspace/test'
    root_manager.update_root(path, set_data_path=False)
    analyst._context.user_command.message = "Form a flock with other robots, navigating together by keeping aligned, spaced out, and cohesive. Avoid obstacles and stay away from the environment's edges and obstacles."
    # analyst.context.load_from_file(f'{path}/analyze_functions_stage.pkl')
    # analyst.context.load_from_file(f'{path}/analyze_constraints_stage.pkl')

    asyncio.run(analyst.run())
    # analyst.context.save_to_file(f'{path}/analyze_constraints_stage.pkl')
    analyst.context.save_to_file(f'{path}/analyze_functions_stage.pkl')
    # analyst.context.save_to_file(f'{path}/analyze_parameters_stage.pkl')
