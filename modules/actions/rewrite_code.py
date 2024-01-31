from const import WORKSPACE_ROOT
from modules.actions.action import Action
from modules.utils import parse_code, write_file
from modules.utils import read_file

PROMPT_TEMPLATE = """
This is the source file you previously wrote, but there are error messages as follows. 
code:
```python
{code}
```
Error message:
{error_message}

The source file is at the same directory level as the environment file (filename=env.py).
Use 'from env import env' to import the environment file.


You need to rewrite the entire source file to correct the errors. 
Note: output the complete file.

you should correctly import the necessary classes based on these file locations!

Return ```python your_code_here ``` with NO other texts,
your code:
"""


class RewriteCode(Action):
    name: str = "RewriteCode"

    async def _write_code(self, prompt):
        code_rsp = await self._ask(prompt)

        try:
            code = parse_code(text=code_rsp)
        except Exception:
            # Handle the exception if needed
            self._logger.error(f"Can't parse the code: {code_rsp}")

            # Return code_rsp in case of an exception, assuming llm just returns code as it is and doesn't wrap it inside ```
            code = code_rsp
        return code

    async def _save(self, filename, code):
        code_path = WORKSPACE_ROOT
        write_file(directory=code_path, filename=filename, content=code)
        self._logger.info(f"Saving Code to {code_path}/{filename}")

    async def run(self, content):
        file_name = content['file_name']
        error_message = content['instruction']
        code = read_file(WORKSPACE_ROOT, file_name)
        prompt = PROMPT_TEMPLATE.format(
                code=code,
                error_message=error_message,
                )
        code = await self._write_code(prompt)
        await self._save(file_name, code)
        return code
