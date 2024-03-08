from abc import ABC, abstractmethod
from collections import namedtuple

from modules.framework.workflow_context import WorkflowContext
from modules.utils import setup_logger, LoggerLevel, format_log_message
from modules.llm.gpt import GPT
from tenacity import retry, stop_after_attempt, wait_random_exponential

ActionResult = namedtuple('ActionResult', ['id', 'message'])


class Action(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)
        self._llm = GPT()
        self._context = WorkflowContext()

    def __str__(self):
        return self.__class__.__name__

    def set_prefix(self, prefix):
        self._llm.system_prompt = prefix
        self._llm.reset()

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    async def run(self, **kwargs) -> str:
        res = await self._run(**kwargs)
        res = self.process_response(res, **kwargs)
        return res

    @abstractmethod
    def process_response(self, response: str, **kwargs) -> str:
        return response

    async def _run(self, **kwargs) -> str:
        prompt = kwargs.get('prompt', None)
        if prompt is None:
            raise SystemExit("Prompt is required")
        code = await self._ask(prompt)
        return code

    async def _ask(self, prompt: str) -> str:
        result = await self._llm.ask(prompt)
        # store PROMPT and RESULT in the log.md
        # make sure output them after _llm.ask(), for it's an asynchronize function
        self._context.log.format_message(prompt,"prompt")
        self._context.log.format_message(result,"response")
        return result


if __name__ == "__main__":
    action = Action()
    import asyncio

    asyncio.run(action.run())
    # story = action.llm.ask("tell a story")
