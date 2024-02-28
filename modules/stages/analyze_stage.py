from modules.stages.stage import Stage, StageResult
from modules.actions import WritePrompt
from const import ENV_CODE


PROMPT_TEMPLATE: str = """
env_code:
{code}

user requirement: {instruction}
Enrich and organize user requirements based on existing simulation environment code and according to the following basic format.
1)task objectives:
2)core requirements:
3)constraints:
4)task success criteria:
"""

class AnalyzeStage(Stage):
    def __init__(self, action: WritePrompt):
        super(AnalyzeStage, self).__init__()
        self._user_command = ""
        self._action = action

    def _run(self) -> StageResult:
        prompt = PROMPT_TEMPLATE.format(instruction=self._context.user_command, code=ENV_CODE) 
        self._context.analysis  = self._action.run(prompt=prompt)
        return StageResult(keys=[])

if __name__ == "__main__":
    stage = AnalyzeStage(WritePrompt())
    res = stage.run()
    print(res)