import asyncio

from modules.stages.stage import Stage, StageResult
from modules.actions import Analyze
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.analyze_stage_prompt import ANALYZE_PROMPT_TEMPLATE
from modules.prompt.task_description import TASK_DES


class AnalysisStage(Stage):
    def __init__(self, action: Analyze):
        super(AnalysisStage, self).__init__()
        self._user_command = ""
        self._action = action

    async def _run(self) -> StageResult:
        prompt = ANALYZE_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self._context.user_command.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES
        )
        await self._action.run(prompt=prompt)
        return StageResult(keys=[])


if __name__ == '__main__':
    analyst = AnalysisStage(Analyze())
    analyst._context.user_command.message = 'Form a flocking formation with other robots, maintaining a 0.5m distance between each robot, moving as quickly as possible, and avoiding collisions with environmental boundaries or obstacles.'
    asyncio.run(analyst.run())
