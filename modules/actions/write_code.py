from modules.actions.action import Action
from modules.utils import parse_code, extract_imports_and_functions, extract_top_level_function_names
from modules.framework.workflow_context import WorkflowContext


class WriteCode(Action):
    name: str = "WriteCode"

    def __init__(self, ):
        super().__init__()

    def process_response(self, response: str, **kwargs) -> str:
        if "function_name" in kwargs:
            desired_function_name = kwargs.get("function_name")
        else:
            raise SystemExit("Function name is not provided")
        code = parse_code(text=response)
        function_list = extract_top_level_function_names(code)
        if not function_list:
            self._context.log.format_message(f"Write Code Failed: No function detected in the response", "error")
            raise Exception
        if len(function_list) > 1:
            self._context.log.format_message(f"Write Code Failed: More than one function detected in the response", "error")
            raise Exception
        for function_name in function_list:
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                self._context.log.format_message(f"Write Code Failed: No function detected in the response", "error")
                raise Exception
        self._context.function_pool.add_functions(content=code)
        return code
