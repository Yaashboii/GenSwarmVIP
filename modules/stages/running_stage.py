from modules.stages.stage import Stage, StageResult
from modules.actions import RunCode


class RunningStage(Stage):
    def __init__(self, action: RunCode = None):
        super().__init__()
        self._action = action

    def _run_code(self):
        code_info = {
            'command': ["python", "run.py"]
        }
        result = self._action.run(
            code_info=code_info,
            mode='script',
        )
        return result

    async def _run(self) -> StageResult:
        result = await self._run_code()
        self._context.run_result.message = result
        return StageResult(keys=[])
