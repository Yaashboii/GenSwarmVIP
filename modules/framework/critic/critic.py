import logging
from abc import ABC, abstractmethod

class Critic(ABC):
    """
    The Critic class is an abstract base class representing a generic critic.
    """

    def __init__(self) -> None:
        """
        Initializes a Critic instance with a logger set to DEBUG level.
        """
        self._logger = logging.getLogger(self.__class__.__name__)
        self._logger.setLevel(logging.DEBUG)

    @abstractmethod
    def evaluate(self) -> (bool, str):
        """
        Abstract method to perform the evaluation.

        Returns:
            str: Suggestions or feedback based on the evaluation.
        """
        pass


