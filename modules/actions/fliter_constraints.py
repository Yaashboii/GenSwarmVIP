from modules.actions.action import Action
from modules.utils import parse_code


class FilterConstraints(Action):
    name: str = "FilterConstraints"

    def __init__(self, ):
        super().__init__()

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response, lang='json')
        return code
