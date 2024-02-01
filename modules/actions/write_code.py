import json

from modules.actions.action import Action
from const import WORKSPACE_ROOT, ENV_CODE
from modules.utils import write_file, parse_code
from tenacity import retry, stop_after_attempt, wait_random_exponential

PROMPT_TEMPLATE = """
Based on existing code and user requirements, write a separate algorithm file.
requirements:
{instruction}
current code(env.py):
{code}
Write code with triple quoto, based on the following attentions and context.
1. Only One file: do your best to implement THIS ONLY ONE FILE.
2. COMPLETE CODE: Your code will be part of the entire project, so please implement complete, reliable, reusable code snippets.
3. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. AVOID circular import.
4. CAREFULLY CHECK THAT YOU DONT MISS ANY NECESSARY CLASS/FUNCTION IN THIS FILE.
5. Before using a external variable/module, make sure you import it first.
6. Write out EVERY CODE DETAIL, DON'T LEAVE TODO.
8. Place the algorithm code in a separate file, and you must import the existing 'env' instance by 'from env import env'.
Return ```python your_code_here ``` with NO other texts,
your code:
"""


class WriteCode(Action):
    name: str = "WriteCode"

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    async def _write_code(self, prompt):
        code_rsp = await self._ask(prompt)
        code = parse_code(text=code_rsp, lang='python')
        return code

    async def _save(self, filename, code):
        code_path = WORKSPACE_ROOT
        write_file(directory=code_path, filename=filename, content=code)
        self._logger.info(f"Saving Code to {code_path}/{filename}")

    async def run(self, instruction, filename='core.py'):
        prompt = PROMPT_TEMPLATE.format(instruction=instruction, code=ENV_CODE)
        self._logger.info(f'Writing {filename}..')
        code = await self._write_code(prompt)
        # code_rsp = await self._aask_v1(prompt, "code_rsp", OUTPUT_MAPPING)
        await self._save(filename, code)
        result = {
            "code":     code,
            "filename": filename
            }
        return str(result)
