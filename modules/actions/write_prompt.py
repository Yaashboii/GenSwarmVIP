from modules.actions.action import Action, ActionResult
from const import ENV_CODE


class WritePrompt(Action):
    PROMPT_TEMPLATE: str = """
env_code:
{code}

user requirement: {instruction}
Enrich and organize user requirements based on existing simulation environment code and according to the following basic format.
1)task objectives:
2)core requirements:
3)constraints:
4)task success criteria:
"""
    name: str = "WritePrompt"

    def _run(self, action_result: ActionResult) -> ActionResult:
        prompt = self.PROMPT_TEMPLATE.format(instruction=action_result.message, code=ENV_CODE)
        context = self._ask(prompt)
        return ActionResult(0, context)