from modules.actions import Action
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

    async def run(self, instruction: str) -> str:
        prompt = self.PROMPT_TEMPLATE.format(instruction=instruction, code=ENV_CODE)

        context = await self._ask(prompt)

        return context
