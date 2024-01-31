import logging
import ast
import re
import json

from modules.roles.role import Role
from modules.actions import WriteCode, WriteDesign, RunCode, RewriteCode, WriteRun, ReWriteRun
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
        self._init_actions([WriteCode, WriteRun, RewriteCode, ReWriteRun])
        self._watch([WriteDesign, RunCode])

    async def _think(self, msg):
        if msg.cause_by == 'WriteDesign':
            self.next_action = self.actions['WriteCode']
        elif msg.cause_by == 'RunCode':
            if "code.py" has error:
                self.next_action = self.actions['RewriteCode']
            elif code.py no error:
                self.next_action = self.actions['WriteRun']
            elif run.py has error:
                self.next_action = self.actions['ReWriteRun']
            else:
                self._looger.info("final code is finished.")
    # async def _act(self, msg) -> Message:
    #     rsp = await self.next_action.run(msg.content)
    #     msg = Message(content=rsp, role=self.profile,
    #                   cause_by=self.next_action, sent_from=self)
    #     return msg
