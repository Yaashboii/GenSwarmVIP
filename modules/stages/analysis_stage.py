import asyncio

from modules.stages.stage import Stage, StageResult
from modules.actions import Analyze, DesignParameters
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.analyze_stage_prompt import ANALYZE_PROMPT_TEMPLATE, PARAMETER_PROMPT_TEMPLATE
from modules.prompt.task_description import TASK_DES


class AnalysisStage(Stage):
    def __init__(self, action: Analyze):
        super(AnalysisStage, self).__init__()
        self._user_command = ""
        self._prompt = None
        self._action = action

    async def _analyze_requirements(self):
        self._prompt = ANALYZE_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self._context.user_command.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES
        )
        self._action = Analyze()
        await self._action.run(prompt=self._prompt)

    async def _design_parameters(self):
        self._action = DesignParameters()
        self._prompt = PARAMETER_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            requirements_constraints=self.context.analysis.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES
        )
        await self._action.run(prompt=self._prompt)

    async def _run(self) -> StageResult:
        await self._analyze_requirements()
        # await self._design_parameters()
        return StageResult(keys=[])


if __name__ == '__main__':
    analyst = AnalysisStage(Analyze())
    from modules.utils import root_manager

    path = '/home/derrick/catkin_ws/src/code_llm/workspace/test'
    root_manager.update_root(path, set_data_path=False)
    analyst._context.user_command.message = 'Form a flocking formation with other robots, maintaining a 0.5m distance between each robot, moving as quickly as possible, and avoiding collisions with environmental boundaries or obstacles.'
    asyncio.run(analyst.run())
