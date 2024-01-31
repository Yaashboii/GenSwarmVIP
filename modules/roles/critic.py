from modules.roles.role import Role
from modules.actions import RunCode, DebugError, WriteUnitTest, WriteCode, WriteRun, RewriteUnitTest, RewriteCode
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
        self._watch([WriteCode, DebugError, WriteRun, RewriteCode])

    async def _think(self, msg):
        if msg.cause_by in ['WriteCode', 'WriteRun', 'ReWriteCode']:
            self.next_action = self.actions['RunCode']
    
    async def _act(self, msg) -> Message:
        is_passed, res_msg = await self._act_run_code(msg.content)
        if not is_passed: return res_msg
        self.next_action = self.actions['WriteUnitTest']
        code = await self.next_action.run(msg.content)
        is_passed, res_msg = await self._act_run_code(code)
        if not is_passed: return res_msg
        return Message(content="No actions taken yet", cause_by=Action)
    
    async def _act_run_code(self, code) -> (bool, Message):
        is_passed, response = await self.actions['RunCode'].run(code)
        res_msg =  Message(content=response, role=self.profile,
                          cause_by=self.next_action, sent_from=self)
        return is_passed, res_msg