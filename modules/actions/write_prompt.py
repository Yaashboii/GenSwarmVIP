from modules.actions.action import Action


class WritePrompt(Action):
    name: str = "WritePrompt"

    def _run(self, prompt: str) -> str:
        # prompt = self.PROMPT_TEMPLATE.format(instruction=action_result.message, code=ENV_CODE)
        context = self._ask(prompt)
        return context