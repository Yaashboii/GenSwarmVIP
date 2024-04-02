from modules.actions.action import Action
from modules.utils import parse_code


class Debug(Action):
    name: str = 'Debug'

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        self._context.function_pool.add_functions(content=code)
        return str(code)
