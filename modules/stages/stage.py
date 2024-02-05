from enum import Enum
from typing import List, Union, final
from abc import ABC, abstractmethod

from pydantic import BaseModel, Field

from modules.utils.logger import setup_logger
from modules.utils.common import TestResult, BugSource
from modules.framework.workflow_context import WorkflowContext

class StageResult(BaseModel):
    keys: List[Union[TestResult, BugSource]] = Field(default=[])

class StageType(Enum):
    AnalyzeStage = 1
    DesignStage = 2
    CodingStage = 3
    TestingStage = 4
    FinalStage = 5

class Stage(ABC, BaseModel):
    def __init__(self):
        super(Stage, self).__init__()
        self._logger = setup_logger(self.__class__.__name__)
        self._context = WorkflowContext()

    def __str__(self) -> str:
        return self.__class__.__name__
    
    @final
    def run(self) -> StageResult:
        self._logger.info(f"Current stage: {self}")
        return self._run()
    
    def _run(self) -> StageResult:
        return StageResult()