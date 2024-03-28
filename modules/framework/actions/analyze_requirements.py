from modules.framework.action import ActionNode


class AnalyzeReqs(ActionNode):
    def _process_response(self, response: str) -> str:
        self._context.analysis.message = response
        self._context.log.format_message(f"Analyze Requirements Success", "success")
        return response
