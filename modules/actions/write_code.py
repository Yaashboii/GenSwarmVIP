from modules.actions.action import Action
from modules.utils import parse_code, extract_imports_and_functions


class WriteCode(Action):
    name: str = "WriteCode"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        if not kwargs.get('filename'):
            self._logger.error(f"Write Sequence Diagram Failed: No filename provided")
            raise SystemExit  # avoid retry mechanism

        filename = kwargs.get('filename')
        if filename not in self._context.code_files:
            self._logger.error(f"Write Code Failed: No filename found in context")
            raise SystemExit

        elif filename == "functions.py":
            import_list, function_list = extract_imports_and_functions(code)
            if len(function_list) > 1:
                self._logger.error(
                    f"Write Code Failed: More than one function detected in the response: {function_list}")
                raise Exception  # to trigger retry
            result = {
                "import": import_list,
                "code": function_list
            }
            return str(result)
        elif filename == "run.py":
            self._context.code_files[filename].message = 'from functions import *\n' + code
            return code
