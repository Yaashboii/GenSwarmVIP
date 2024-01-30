from modules.roles.role import Role
from modules.actions import RunCode, DebugError, WriteTest, WriteCode
from modules.framework.message import Message
from modules.actions.action import Action

class Architect(Role):
    name: str = "Edward"
    profile: str = "Architect"
    goal: str = ("Write comprehensive and robust tests to ensure codes will work"
                 " as expected without bugs")
    constraints: str = (
        "The test code you write should conform to code standard like PEP8, be "
        "modular, easy to read and maintain"
    )
    
    def __init__(self, **data) -> None:
        super().__init__(**data)
        # self._init_actions([RunCode, WriteTest])
        # self._watch([WriteCode, DebugError])

    async def _think(self, msg):
        pass
    
    async def _act(self, msg) -> Message:
        return Message(content="No actions taken yet", cause_by=Action)
