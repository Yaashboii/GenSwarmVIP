from modules.actions.action import Action

PROMPT_TEMPLATE = """
write test function in python for the following code:

{code}
"""

class WriteTest(Action):
    name: str= "WriteTest"
        
    async def run(self, code) -> str:
        await super().run()
        prompt = PROMPT_TEMPLATE.format(code=code)
        res = self._ask(prompt)
        self._logger.debug("res: %s", res)
        return res

if __name__ == "__main__":
    action = WriteTest()
    import asyncio
    asyncio.run(action.run())