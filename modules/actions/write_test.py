import json

from modules.actions.action import Action
from modules.utils import write_file, parse_code
from const import WORKSPACE_ROOT


class WriteUnitTest(Action):
    name: str = "WriteUnitTest"
    
    def __init__(self):
        super().__init__()

    def _write_code(self, prompt):
        code_rsp = self._ask(prompt)

        try:
            code = parse_code(text=code_rsp)
        except Exception:
            # Handle the exception if needed
            self._logger.error(f"Can't parse the code: {code_rsp}")

            # Return code_rsp in case of an exception, assuming llm just returns code as it is and doesn't wrap it inside ```
            code = code_rsp
        return code

    def _run(self, prompt: str, filename: str) -> str:
        code =  self._write_code(prompt)
        write_file(filename, code)
        command = ["python", f"{filename}"]
        return command