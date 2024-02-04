from enum import Enum
from typing import List, Any
from abc import ABC, abstractmethod

from typing import final
from modules.utils.logger import setup_logger


class StageType(Enum):
    AnalyzeStage = 1
    DesignStage = 2
    WriteCoreStage = 3
    TestCoreStage = 4
    WriteMainStage = 5
    TestMainStage = 6
    FinalStage = 7
    RunTestStage = 8

class Stage(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__)

    def __str__(self) -> str:
        return self.__class__.__name__
    
    @abstractmethod
    def update(self):
        raise NotImplementedError
    
    @final
    def run(self) -> (str, List[Any]):
        self._logger.info(f"Current stage: {self}")
        return self._run()
    
    def _run(self) -> (str, List[Any]):
        return "", []