from typing import List, Any

from modules.stages.stage import Stage
from modules.actions import ActionResult, WriteDesign


class DesignStage(Stage):
    def __init__(self,  action: WriteDesign):
        super().__init__()
        self._analysis = ""
        self._action = action

    def update(self, data: str):
        self._analysis = data

    def _run(self) -> (str, List[Any]):
        res = self._action.run(ActionResult(0, self._analysis))
        return (res.message, [])