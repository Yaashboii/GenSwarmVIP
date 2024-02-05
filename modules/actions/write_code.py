import json

from modules.actions.action import Action
from const import WORKSPACE_ROOT
from modules.utils import write_file, parse_code
from tenacity import retry, stop_after_attempt, wait_random_exponential

class WriteCode(Action):
    name: str = "WriteCode"

    def __init__(self):
        super().__init__()

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    def _write_code(self, prompt):
        code_rsp = self._ask(prompt)
        code = parse_code(text=code_rsp, lang='python')
        return code

    def _run(self, prompt: str, filename: str) -> str:
        self._logger.info("Writing %s..", filename)
        # code = self._write_code(prompt)
        # code_rsp = self._aask_v1(prompt, "code_rsp", OUTPUT_MAPPING)
        # write_file(filename, code)
        return "code"


if __name__ == "__main__":
    action = WriteCode()
    action.run(prompt="prompt", filename="filename")