from modules.actions.action import Action
from modules.utils import parse_code, extract_function_definitions, extract_top_level_function_names


class DesignFunction(Action):
    name: str = "DesignFunction"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        function_list = extract_function_definitions(code)
        if not function_list:
            self._context.log.formssage(f"Design Function Failed: No function detected in the response", "error")
            raise Exception  # trigger retry
        if len(function_list) > 1:
            self._context.log.formssage(f"Design Function Failed: More than one function detected in the response",
                                        "error")
            raise Exception  # trigger retry
        for function in function_list:
            function_name = extract_top_level_function_names(code_str=function)[0]
            if not function_name:
                self._context.log.format_message(f"Design Function Failed: No function detected in the response",
                                                 "error")
                raise Exception  # trigger retry
            self._context.function_pool.functions[function_name].content = function
        # self._context.log.format_message(f"{code}", "response")
        return str(code)
