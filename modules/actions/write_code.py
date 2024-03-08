from modules.actions.action import Action
from modules.utils import parse_code, extract_imports_and_functions


class WriteCode(Action):
    name: str = "WriteCode"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        if not kwargs.get('filename'):
            self._context.log.format_message(f"Write Sequence Diagram Failed: No filename provided", "error")
            raise SystemExit  # avoid retry mechanism

        filename = kwargs.get('filename')
        if filename not in self._context.code_files:
            self._context.log.format_message(f"Write Code Failed: No filename found in context","error")
            raise SystemExit

        elif filename == "functions.py":
            import_list, function_list = extract_imports_and_functions(code)
            if not function_list:
                self._context.log.format_message(f"Write Code Failed: No function detected in the response","error")
                raise Exception
            elif len(function_list) > 1:
                self._context.log.format_message(f"Write Code Failed: More than one function detected in the response: {function_list}","error")
                raise Exception  # to trigger retry

            result = {
                "import": import_list,
                "code": function_list
            }
            return str(result)
        elif filename == "run.py":
            code_prefix = "from functions import * \nimport sys\n\nos.environ['ROBOT_ID'] = sys.argv[1]\n"
            self._context.code_files[filename].message = code_prefix + code
            return code
