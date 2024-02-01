import logging
import ast
import re
import json

from modules.roles.role import Role
from modules.actions import WriteCode, WriteDesign, RunCode, RewriteCode, WriteRun, RewriteRun
from modules.framework.message import Message


class Actor(Role):
    name: str = "Alex"
    profile: str = "Engineer"
    goal: str = "write elegant, readable, extensible, efficient code"
    constraints: str = (
        "the code should conform to standards like google-style and be modular and maintainable. "
        "Use same language as user requirement"
    )

    def __init__(self, **data) -> None:
        super().__init__(**data)
        self._init_actions([WriteCode, WriteRun, RewriteCode, RewriteRun])
        self._watch([WriteDesign, RunCode])

    async def _think(self, msg):
        if msg.cause_by == 'WriteDesign':
            self.next_action = self.actions['WriteCode']
        elif msg.cause_by == 'RunCode':
            result = eval(msg.content)
            if result["file_name"] == "core.py":
                self.next_action = self.actions['RewriteCode']
            elif result["file_name"] == "":
                self.next_action = self.actions['WriteRun']
            elif result["file_name"] == "run.py":
                self.next_action = self.actions['RewriteRun']