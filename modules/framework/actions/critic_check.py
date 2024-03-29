from modules.framework.action import ActionNode


class CriticCheck(ActionNode):
    def _process_response(self, response: str) -> str:
        return response
    
    async def _run(self) -> str:
        if not self._prompt:
            raise ValueError("Prompt must not be empty")
        res = await self._ask(self._prompt)
        return res
