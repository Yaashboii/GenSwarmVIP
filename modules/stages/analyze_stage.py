from typing import List, Any

from modules.stages.stage import Stage
from modules.actions import WritePrompt, ActionResult


class AnalyzeStage(Stage):
    def __init__(self, action: WritePrompt):
        super(AnalyzeStage, self).__init__()
        self._user_command = ""
        self._action = action

    def update(self, data: str):
        self._user_command = data

    def _run(self) -> (str, List[Any]):
        res = self._action.run(ActionResult(0, self._user_command))
        return (res.message, [])

if __name__ == "__main__":
    stage = AnalyzeStage()
    res = stage.run()
    print(res)