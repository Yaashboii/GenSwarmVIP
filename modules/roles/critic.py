from modules.roles.role import Role
from modules.actions import RunCode, WriteUnitTest, WriteCode, WriteRun, RewriteUnitTest, RewriteCode, RewriteRun
from modules.framework.message import Message
from modules.actions.action import Action


class Critic(Role):
    name: str = "Edward"
    profile: str = "QaEngineer"
    goal: str = ("Write comprehensive and robust tests to ensure codes will work"
                 " as expected without bugs")
    constraints: str = (
        "The test code you write should conform to code standard like PEP8, be "
        "modular, easy to read and maintain"
    )

    def __init__(self, **data) -> None:
        super().__init__(**data)
        self._init_actions([RunCode, WriteUnitTest, RewriteUnitTest])
        self._watch([WriteCode, WriteRun, RewriteCode, RewriteRun, RewriteUnitTest])

    async def _think(self, msg):
        # TODO: action name might be misspelled, change data type 
        if msg.cause_by in ['WriteCode', 'WriteRun']:
            self.next_action = self.actions['WriteUnitTest']
        elif msg.cause_by in ['RewriteCode', 'RewriteRun', 'RewriteUnitTest']:
            self.next_action = self.actions['RunCode']
        elif msg.cause_by in ['RunCode']:
            self.next_action = self.actions['RewriteUnitTest']

    async def _act(self, msg) -> Message:
        if msg.cause_by in ['WriteCode', 'WriteRun']:
            code_info = await self.next_action.run(msg)
            rsp = await self.actions['RunCode'].run(code_info)
        elif msg.cause_by in ['RewriteCode', 'RewriteRun', 'RewriteUnitTest']:
            rsp = await self.next_action.run(msg.content)
        elif msg.cause_by in ['RunCode']:
            rsp = await self.next_action.run(msg)
        else:
            rsp = ''

        if isinstance(rsp, Message):
            msg = rsp
        else:
            msg = Message(content=rsp, role=self.profile,
                          cause_by=self.next_action, sent_from=self)
        return msg
