from modules.actions.action import Action


class RunCode(Action):
    name: str= "RunCode"
        
    async def run(self, code) -> (bool, str):
        await super().run()
        self._logger.debug("run your code: \n%s", code )
        return (True, "I run your code")
        exec()

if __name__ == "__main__":
    action = RunCode()
    import asyncio
    asyncio.run(action.run())