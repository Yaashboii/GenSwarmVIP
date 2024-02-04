import json

from modules.actions.action import Action, ActionResult
from const import WORKSPACE_ROOT
from modules.utils import write_file, parse_code
from tenacity import retry, stop_after_attempt, wait_random_exponential

class WriteCode(Action):
    name: str = "WriteCode"

    def __init__(self, filename, prompt_template=""):
        super().__init__()
        self.filename = filename
        self.prompt_template = prompt_template

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    def _write_code(self, prompt):
        code_rsp = self._ask(prompt)
        code = parse_code(text=code_rsp, lang='python')
        return code

    def _save(self, filename, code):
        code_path = WORKSPACE_ROOT
        write_file(directory=code_path, filename=filename, content=code)
        self._logger.info("Saving Code to %s/%s", code_path, filename)
        print("=====")

    def _run(self, action_result: ActionResult):
        prompt = action_result.message
        self._logger.info("Writing %s..", self.filename)
        code = self._write_code(prompt)
        # code_rsp = self._aask_v1(prompt, "code_rsp", OUTPUT_MAPPING)
        self._save(self.filename, code)
        return ActionResult(0, code)
