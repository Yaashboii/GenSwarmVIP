from modules.roles.role import Role
from modules.actions import WritePrompt, UserCommand


class Analyst(Role):
    name: str = "Tony"
    profile: str = "Analyst"
    goal: str = ("?")
    constraints: str = ()
    
    def __init__(self, **data) -> None:
        super().__init__(**data)
        self._init_actions([WritePrompt])
        self._watch([UserCommand])