from abc import ABC, abstractmethod

from modules.utils import setup_logger, LoggerLevel


class CodeError(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)


class Bug(CodeError):
    def __init__(self, error_msg):
        super().__init__()
        self.error_msg = error_msg


class CriticNotSatisfied(CodeError):
    pass


class HumanFeedback(CodeError):
    def __init__(self, feedback):
        super().__init__()
        self.feedback = feedback
