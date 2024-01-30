from modules.actions.action import Action

PROMPT_TEMPLATE = """
write python code according to following requirements

{requirement}
"""

class WriteRun(Action):
    name: str= "WriteRun"
        
    async def run(self, input) -> str:
        await super().run()
        prompt = PROMPT_TEMPLATE.format(requirement=input)
        res = self._ask(prompt)
        self._logger.debug("res: %s", res)
        return res

if __name__ == "__main__":
    action = WriteRun()
    import asyncio
    asyncio.run(action.run())