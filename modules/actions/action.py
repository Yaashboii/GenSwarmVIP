from pydantic import BaseModel, ConfigDict, Field, model_validator

from modules.utils.logger import setup_logger, LoggerLevel
from modules.llm.gpt import GPT


class Action(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True, exclude=["llm"])

    name: str = ""
    llm: GPT = Field(default_factory=GPT, exclude=True)

    @model_validator(mode="before")
    def set_name_if_empty(cls, values):
        if "name" not in values or not values["name"]:
            values["name"] = cls.__name__
        return values

    def __init__(self, **data):
        super().__init__()
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)

    def __str__(self):
        return self.__class__.__name__

    async def _ask(self, prompt: str) -> str:
        return self.llm.ask(prompt)

    def set_prefix(self, prefix):
        self.llm.system_prompt = prefix

    async def run(self, *args, **kwargs) -> str:
        self._logger.info("Running action")


if __name__ == "__main__":
    action = Action()
    import asyncio

    asyncio.run(action.run())
    # story = action.llm.ask("tell a story")
