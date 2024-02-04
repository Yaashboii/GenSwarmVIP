from tenacity import retry, stop_after_attempt, wait_random_exponential

from modules.actions.action import Action, ActionResult
from const import WORKSPACE_ROOT, ENV_CODE
from modules.utils import write_file, parse_code

PROMPT_TEMPLATE = """
Based on existing code and user requirements,you need to conceptualize how to design an algorithm to independently achieve the objective, output the corresponding class diagram, and the call graph of the algorithm.
Our project is divided into three files in total. Among them, `env.py` represents the pre-defined simulation environment conditions. 
`core.py` constitutes the core algorithms that fulfill user requirements.
`run.py` serves as an interface to the environment, directly executing the algorithms, and is the file where the user requirements are ultimately realized.
Now, you need to design a class diagram for this system, along with a diagram depicting the calling relationships.
requirements:
{instruction}
current code(env.py):
{code}

Use Mermaid's sequenceDiagram. and classDiagram.

Return ```mermaid  ``` with NO other texts,
"""

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

    def _save(self, filename, code):
        code_path = WORKSPACE_ROOT
        write_file(directory=code_path, filename=filename, content=code)
        self._logger.info(f"Saving design result to {code_path}/{filename}")

    def _run(self, action_result: ActionResult) -> ActionResult:
    # async def _run(self, instruction, filename='design.txt'):
        filename='design.md'
        prompt = PROMPT_TEMPLATE.format(instruction=action_result.message, code=ENV_CODE)
        self._logger.info(f'Writing {filename}..')
        code = self._write_design(prompt)
        self._save(filename, DESIGN_PATTERN.format(code=code))
        return ActionResult(0, code)
