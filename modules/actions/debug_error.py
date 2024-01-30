from modules.actions.action import Action

PROMPT_TEMPLATE = """
{code}

{error_msg}
"""

class DebugError(Action):
    name: str= "DebugError"
        
    async def run(self) -> str:
        await super().run()
        return "I debug your error"
        prompt = PROMPT_TEMPLATE.format(requirement="write python code")
        res = self._ask(prompt)
        self._logger.debug("res: %s", res)

if __name__ == "__main__":
    action = DebugError()
    import asyncio
    asyncio.run(action.run())