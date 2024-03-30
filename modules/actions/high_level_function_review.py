from modules.actions.action import Action
from modules.utils import parse_code, extract_function_definitions, extract_top_level_function_names


class HighLevelFunctionReview(Action):
    name: str = "HighLevelFunctionReview"

    def process_response(self, response: str, **kwargs) -> str:
        try:
            if 'function_name' in kwargs:
                desired_function_name = kwargs.get("function_name")
            else:
                raise SystemExit("Function name is not provided")
            code = parse_code(text=response)
            function_list = extract_function_definitions(code)
            if not function_list:
                self._context.log.format_message(
                    f"High Level Function Review Failed: No function detected in the response",
                    "error")
                return ''
            if len(function_list) > 1:
                self._context.log.format_message(
                    f"High Level Function Review Failed: More than one function detected in the response",
                    "error")
                raise Exception(f"More than one function detected in the response")
            function_name = extract_top_level_function_names(code_str=code)[0]
            if function_name != desired_function_name:
                self._context.log.format_message(
                    f"High Level Function Review Failed: Function name mismatch: {function_name} != {desired_function_name}",
                    "error")
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            self._context.function_pool.add_functions(content=code)
            return code
        except ValueError as e:
            self._context.log.format_message(f"No function detected in the response: {e}", 'warning')
        except Exception as e:
            self._context.log.format_message(f"High Level Function Review Failed: {e}", "error")
            raise Exception  # trigger retry
