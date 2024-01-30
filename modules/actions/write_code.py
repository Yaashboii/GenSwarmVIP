from modules.actions.action import Action

PROMPT_TEMPLATE = """
write python code according to following requirements

{requirement}
"""

class WriteCode(Action):
    name: str= "WriteCode"
        
    async def run(self, input) -> str:
        await super().run()
        prompt = PROMPT_TEMPLATE.format(requirement=input)
        res = self._ask(prompt)
        self._logger.debug("res: %s", res)
        return res

if __name__ == "__main__":
    action = WriteCode()
    import asyncio
    asyncio.run(action.run())