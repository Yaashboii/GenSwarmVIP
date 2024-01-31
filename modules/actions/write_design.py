from modules.actions.action import Action
from const import WORKSPACE_ROOT, ENV_CODE
from modules.utils import write_file, parse_code
from tenacity import retry, stop_after_attempt, wait_random_exponential

PROMPT_TEMPLATE = """
Based on existing code and user requirements,you need to conceptualize how to design an algorithm to independently achieve the objective, output the corresponding class diagram, and the call graph of the algorithm.

requirements:
{instruction}
current code(env.py):
{code}

Use Mermaid's sequenceDiagram. and classDiagram.

Return ```mermaid  ``` with NO other texts,
"""


class WriteDesign(Action):
    name: str = "WriteDesign"

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    async def write_design(self, prompt):
        code_rsp = await self._aask(prompt)
        code = parse_code(text=code_rsp, lang='mermaid')
        return code

    def _save(self, filename, code):
        code_path = WORKSPACE_ROOT
        write_file(directory=code_path, filename=filename, content=code)
        self._logger.info(f"Saving Code to {code_path}/{filename}")

    async def run(self, instruction, filename='core'):
        prompt = PROMPT_TEMPLATE.format(instruction=instruction, code=ENV_CODE)
        self._logger.info(f'Writing {filename}..')
        code = await self.write_code(prompt)
        self._save(filename, code)
        return code
