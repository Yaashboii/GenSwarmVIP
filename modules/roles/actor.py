import logging
import ast
import re
import json

from modules.roles.role import Role
from modules.actions import WriteCode, WriteDesign, RunCode, DebugError, WriteRun
from modules.framework.message import Message


class Actor(Role):
    name: str = "Alex"
    profile: str = "Programmer"
    goal: str = "write elegant, readable, extensible, efficient code"
    constraints: str = (
        "the code should conform to standards like google-style and be modular and maintainable. "
        "Use same language as user requirement"
    )
    
    def __init__(self, **data) -> None:
        super().__init__(**data)
        self._init_actions([WriteCode, DebugError, WriteRun])
        self._watch([WriteDesign, RunCode])

    async def _think(self, msg):
        if msg.cause_by == 'WriteDesign':
            self.next_action = self.actions['WriteCode']
        elif msg.cause_by == 'RunCode':
            is_pass = json.loads(msg)['is_pass']
            if is_pass:
                self.next_action = self.actions['WriteRun']
            else:
                self.next_action = self.actions['DebugError']

    async def _act(self, msg) -> Message:
        rsp = await self.next_action.run(msg.content)
        msg = Message(content=rsp, role=self.profile,
                      cause_by=self.next_action, sent_from=self)
        return msg
    

