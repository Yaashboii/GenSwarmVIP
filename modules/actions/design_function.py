from modules.actions.action import Action
from modules.utils import parse_code, extract_function_definitions


class DesignFunction(Action):
    name: str = "DesignFunction"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        function_list = extract_function_definitions(code)
        self._context.function_list = function_list
        self._logger.info(f"Design Function: {function_list}")
        return str(function_list)
