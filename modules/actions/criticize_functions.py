from modules.actions.action import Action
from modules.utils import parse_code


class CriticizeFunctions(Action):
    name: str = "CriticizeFunctions"

    def process_response(self, response: str, **kwargs) -> str:
        if 'True' in response:
            return response
        elif 'False' in response:
            code = parse_code(text=response)
            self._context.function_pool.add_functions(content=code)
            return response
        else:
            print('Criticize Functions Failed: No True or False detected in the response')
            raise Exception  # trigger retry
