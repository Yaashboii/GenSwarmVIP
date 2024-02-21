from tenacity import retry, stop_after_attempt, wait_random_exponential

from modules.actions.action import Action, ActionResult
from const import WORKSPACE_ROOT
from modules.utils import write_file, parse_code


DESIGN_PATTERN = """```mermaid
{code}
```
"""

class WriteDesign(Action):
    name: str = "WriteDesign"

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    def _write_design(self, prompt):
        code_rsp = self._ask(prompt)
        code = parse_code(text=code_rsp, lang='mermaid')
        return code

    def _run(self, prompt: str) -> str:
    # async def _run(self, instruction, filename='design.txt'):
        filename='design.md'
        # prompt = PROMPT_TEMPLATE.format(instruction=action_result.message, code=ENV_CODE)
        self._logger.info(f'Writing {filename}..')
        code = self._write_design(prompt)
        write_file(filename, DESIGN_PATTERN.format(code=code))
        return code
