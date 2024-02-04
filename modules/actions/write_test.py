import json

from modules.actions.action import Action, ActionResult
from modules.utils import write_file, parse_code
from const import WORKSPACE_ROOT


class WriteUnitTest(Action):
    name: str = "WriteUnitTest"
    
    def __init__(self, filename, prompt_template=""):
        super().__init__()
        self.filename = filename

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

    def _save(self, filename, code):
        code_path = WORKSPACE_ROOT
        write_file(directory=code_path, filename=filename, content=code)
        self._logger.info(f"Saving Code to {code_path}/{filename}")

    def _run(self, action_result: ActionResult) -> ActionResult:
        # content = eval(action_result.message)
        # prompt = self.prompt_template.format(
        #         code_to_test=content["code"],
        #         )
        code =  self._write_code(action_result.message)
        test_file_name = 'test_' + self.filename
        self._save(test_file_name, code)
        command = ["python", f"{test_file_name}"]
        return command