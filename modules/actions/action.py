from abc import ABC, abstractmethod
from collections import namedtuple

from modules.utils.logger import setup_logger, LoggerLevel
from modules.llm.gpt import GPT

ActionResult = namedtuple('ActionResult', ['id', 'message'])

class Action(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)
        self._llm = GPT()

    def __str__(self):
        return self.__class__.__name__

    def set_prefix(self, prefix):
        self._llm.system_prompt = prefix
        self._llm.reset()

    def run(self, **kwargs) -> str:
        res = self._run(**kwargs)
        return res

    @abstractmethod
    def _run(self, **kwargs) -> str:
        pass

    def _ask(self, prompt: str) -> str:
        return self._llm.ask(prompt)

if __name__ == "__main__":
    action = Action()
    import asyncio

    asyncio.run(action.run())
    # story = action.llm.ask("tell a story")
