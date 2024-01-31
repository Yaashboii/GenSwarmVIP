from modules.roles.role import Role
from modules.actions import WriteDesign, RephraseCommand
from modules.framework.message import Message
from modules.actions.action import Action

class Architect(Role):
    name: str = "Bob"
    profile: str = "Architect"
    goal: str = ("design a concise, usable, complete software system")
    constraints: str = (
        "make sure the architecture is simple enough and use  appropriate open source "
        "libraries. Use same language as user requirement"
    )
    
    def __init__(self, **data) -> None:
        super().__init__(**data)
        self._init_actions([WriteDesign])
        self._watch([RephraseCommand])