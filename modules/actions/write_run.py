import json
from tenacity import retry, stop_after_attempt, wait_random_exponential
from modules.actions.action import Action
from const import WORKSPACE_ROOT, ENV_CODE
from modules.utils import write_file, read_file, parse_code

core_code = read_file(directory=WORKSPACE_ROOT, filename='core.py')
user_requirements = read_file(directory=WORKSPACE_ROOT, filename='requirements.py')

PROMPT_TEMPLATE = """
This project has passed the acceptance tests, and now we need to write a core.py for the project. 
The purpose of this core.py is to be executable directly, fulfilling the initial user requirements without requiring any testing. 
I will provide you with some code snippets from the acceptance tests and all the source code references. 
Your task is to generate a correct core.py to the best of your ability.

user_requirements:{user_requirements}
codes:
core.py{core_code}
env.py{env_code}
All of these codes are in the same directory and can be directly called using 'from filename import ....'
You only need to write a very simple runtime code; creating any new algorithms is not allowed. You must invoke other already implemented algorithms. You can refer to the code in the acceptance tests for guidance.

Return ```python your_code_here ``` with NO other texts,
your code:

"""


class WriteRun(Action):
    name: str = "WriteRun"

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    async def _write_code(self, prompt):
        code_rsp = await self._ask(prompt)
        code = parse_code(text=code_rsp)
        return code

    async def _save(self, filename, code):
        code_path = WORKSPACE_ROOT
        write_file(directory=code_path, filename=filename, content=code)
        self._logger.info(f"Saving Code to {code_path}/{filename}")

    async def run(self, filename='run.py'):
        prompt = PROMPT_TEMPLATE.format(user_requirements=user_requirements, env_code=ENV_CODE, core_code=core_code)
        self._logger.info(f'Writing {filename}..')
        code = await self._write_code(prompt)
        await self._save(filename, code)
        result = {
            "code":     code,
            "filename": filename
            }
        return json.dumps(result)
