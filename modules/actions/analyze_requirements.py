from modules.actions.action import Action


class AnalyzeReqs(Action):
    name: str = "AnalyzeReqs"

    def process_response(self, response: str, **kwargs) -> str:
        self._context.analysis.message = response
        self._logger.info(f"Analyze Requirements Success")
        return response
