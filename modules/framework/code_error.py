from abc import ABC, abstractmethod

from modules.utils import setup_logger, LoggerLevel


class CodeError(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)


class Bug(CodeError):
    def __init__(self, error_msg, error_function, error_code):
        super().__init__()
        self.error_msg = error_msg
        self.error_function = error_function
        # error code as prompt
        self.error_code = error_code


class Bugs(CodeError):
    def __init__(self, bug_list: list[Bug], error_code):
        super().__init__()
        self.error_list = bug_list
        # error code as prompt
        self.error_code = error_code
        self.error_msg = ''
        for i, bug in enumerate(bug_list):
            self.error_msg += f"error{i},function_name:{bug.error_function}:" + "\n" + bug.error_msg + "\n"


class CriticNotSatisfied(CodeError):
    pass


class Feedback(CodeError):
    # feedback from human and GPT
    def __init__(self, feedback):
        super().__init__()
        self.feedback = feedback
