import asyncio
from typing import Any, Set

from pydantic import BaseModel, ConfigDict, Field, SerializeAsAny, PrivateAttr

from modules.roles.role import Role
from modules.utils.logger import setup_logger, LoggerLevel
from modules.framework.message import MessageQueue, Message
from modules.actions import UserCommand

class Framework(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    roles: dict[str, SerializeAsAny[Role]] = Field(default_factory=dict, validate_default=True)
    members: list[Role] = Field(default_factory=list, exclude=True)
    command: str = Field(default="")
    _logger = setup_logger("Framework", LoggerLevel.DEBUG)

    def add_roles(self, roles: list[Role]):
        for role in roles:
            self.roles[role.profile] = role
            role.set_framework(self)

    def setup_command(self, command: str):
        self.command = command
        self.publish_message(
            Message(role="Human", content=command, cause_by=UserCommand)
        )

    async def run(self, n_round, command=""):
        if command:
            self.setup_command(command)
        self._logger.debug("command: %s", self.command)
        while n_round > 0:
            n_round -= 1
            self._logger.debug(f"max {n_round=} left.")
            await self._run_roles()
    
    async def _run_roles(self, k=1):
        for _ in range(k):
            futures = []
            for role in self.roles.values():
                future = role.run()
                futures.append(future)

            await asyncio.gather(*futures)
    
    def publish_message(self, message: Message):
        self._logger.debug(f"publish_message: {message.dump()}")
        found = False
        # According to the routing feature plan in Chapter 2.2.3.2 of RFC 113
        for role in self.members:
            role.put_message(message)
            found = True
        if not found:
            self._logger.warning(f"Message no recipients: {message.dump()}")
        return True
        

if __name__ == "__main__":
    
    from modules.roles import Actor
    from modules.roles import Critic
    from modules.roles import Analyst
    from modules.roles import Architect
    from modules.actions import *

    analyst = Analyst()
    actor = Actor()
    critic = Critic()
    architect = Architect()

    framework = Framework()

    framework.add_roles([analyst, actor, critic, architect])
    framework.setup_command("move the car in a circle")
    asyncio.run(framework.run(2))