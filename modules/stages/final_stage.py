from typing import List, Any

from modules.stages.stage import Stage, StageType

class FinalStage(Stage):
    def update(self, data: str):
        self._user_command = data