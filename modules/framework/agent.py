import logging
from abc import ABC, abstractmethod


class Agent(ABC):
    """
    The Agent class is an abstract base class representing an agent
    capable of executing code.
    """
    def __init__(self) -> None:
        self._logger = logging.getLogger("Agent")
        self._logger.setLevel(logging.DEBUG)

    @abstractmethod
    def execute_code(self, code: str):
        """
        Abstract method to execute code. This method must be overridden in
        concrete subclasses.

        Args:
            code (str): The code to be executed.
        """
        self._logger.debug('Executing code: \n%s', code)

class Robot(Agent):
    def __init__(self):
        super(Robot, self).__init__()

    def execute_code(self, code: str):
        return super().execute_code(code)

if __name__ == '__main__':
    logging.basicConfig(level=logging.CRITICAL)
    agent = Robot()
    agent.execute_code("a + b")