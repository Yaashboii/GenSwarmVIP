from modules.actions.action import Action

PROMPT_TEMPLATE = """
Write more detials accoding to following command, in order to get better python code:

{command}
"""

class RephraseCommand(Action):
    name: str= "RephraseCommand"
        
    async def run(self, command) -> str:
        await super().run()
        prompt = PROMPT_TEMPLATE.format(command=command)
        res = self._ask(prompt)
        self._logger.debug("res: %s", res)
        return res

if __name__ == "__main__":
    action = RephraseCommand()
    import asyncio
    asyncio.run(action.run())