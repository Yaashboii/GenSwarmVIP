from modules.actions.action import Action
from modules.utils import parse_code


class DesignParameters(Action):
    name: str = "DesignParameters"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        self._context.parameters.message = code
        self._context.log.format_message(f"Design Parameters success!", "success")
        return response
