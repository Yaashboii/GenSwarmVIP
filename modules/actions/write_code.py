from modules.actions.action import Action
from modules.utils import parse_code, extract_imports_and_functions, extract_top_level_function_names
from modules.framework.workflow_context import WorkflowContext


class WriteCode(Action):
    name: str = "WriteCode"

    def __init__(self, ):
        super().__init__()

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        self._context.function_pool.add_functions(content=code)
        return code
