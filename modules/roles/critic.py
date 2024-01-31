from modules.roles.role import Role
from modules.actions import RunCode, WriteUnitTest, WriteCode, WriteRun, RewriteUnitTest, RewriteCode, ReWriteRun
from modules.framework.message import Message
from modules.actions.action import Action

class Critic(Role):
    name: str = "Edward"
    profile: str = "Tester"
    goal: str = ("Write comprehensive and robust tests to ensure codes will work"
                 " as expected without bugs")
    constraints: str = (
        "The test code you write should conform to code standard like PEP8, be "
        "modular, easy to read and maintain"
    )
    
    def __init__(self, **data) -> None:
        super().__init__(**data)
        self._init_actions([RunCode, WriteUnitTest, RewriteUnitTest])
        self._watch([WriteCode, WriteRun, RewriteCode, ReWriteRun])

    async def _think(self, msg):
        if msg.cause_by in ['WriteCode', 'WriteRun']:
            self.next_action = self.actions['WriteUnitTest']
        elif msg.cause_by in ['ReWriteCode', 'ReWriteRun']:
            self.next_action = self.actions['RunCode']
        elif msg.cause_by in ['RunCode']:
            self.next_action = self.actions['RewriteUnitTest']


    async def _act(self, msg) -> Message:
        if msg.cause_by in ['WriteCode', 'WriteRun']:
            code = await self.next_action.run(msg)
            rsp = await self.actions['RunCode'].run(??)
            return rsp
        elif msg.cause_by in ['ReWriteCode', 'ReWriteRun']:
            rsp = await self.next_action.run(???)
            return rsp
        elif msg.cause_by in ['RunCode']:
            rsp = await self.next_action.run(msg)
            return rsp
        return Message(none???)