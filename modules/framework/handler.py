from abc import ABC, abstractmethod

from modules.utils import setup_logger, LoggerLevel
from modules.framework.action import ActionNode
from modules.framework.code_error import *

class Handler(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)
        self._successor = None
        self._action = None

    @property
    def successor(self):
        return self._successor
    
    @successor.setter
    def successor(self, value):
        self._successor = value

    @property
    def action(self):
        return self._action
    
    @action.setter
    def action(self, value: ActionNode):
        if isinstance(value, ActionNode):
            self._action = value
        else:
            raise TypeError("type of action must be ActionNode")

    @abstractmethod
    def handle(self, request: CodeError):
        if self._action:
            self._action.run()

    def display(self):
        node = self
        content = "Chain of Responsibility: " + self.__class__.__name__
        while node.successor:
            content += f" ==> {node.successor.__class__.__name__}"
            node = node.successor
        self._logger.debug(content)


class BugLevelHandler(Handler):
    def handle(self, request: CodeError):
        if(isinstance(request, Bug)):
            self._logger.debug("Handled by BugLevelHandler")
            super().handle(request)
        elif self._successor:
            self._successor.handle(request)
        return super().handle(request)
    
class CriticLevelHandler(Handler):
    def handle(self, request: CodeError):
        if(isinstance(request, CriticNotSatisfied)):
            self._logger.debug("Handled by CriticLevelHandler")
            super().handle(request)
        elif self._successor:
            self._successor.handle(request)
        return super().handle(request)
    
class HumanFeedbackHandler(Handler):
    def handle(self, request: CodeError):
        if(isinstance(request, HumanFeedback)):
            self._logger.debug("Handled by HumanFeedbackHandler")
            super().handle(request)
        elif self._successor:
            self._successor.handle(request)
        return super().handle(request)

if __name__ == '__main__':
    h1 = BugLevelHandler()
    h2 = CriticLevelHandler()
    h3 = HumanFeedbackHandler()

    h2.successor = h1
    h1.successor = h3

    handle_pipeline = h2

    e1 = Bug()
    e2 = CriticNotSatisfied()
    e3 = HumanFeedback()

    handle_pipeline.display()

    errors = [e1, e2, e3]
    for error in errors:
        handle_pipeline.handle(error)
    