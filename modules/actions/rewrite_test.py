from modules.actions.action import Action
from modules.utils import parse_code, read_file, write_file
from const import WORKSPACE_ROOT

PROMPT_TEMPLATE = """
This is the test file you previously wrote, but there are error messages as follows. 
code:
```python
{test_code}
```
Error message:
{error_message}

The test file is at the same directory level as the source file (filename=core.py).
Use 'from core import ...' and 'from env import env' to import the source file.


You need to rewrite the entire test file to correct the errors. 
Note: output the complete file.


you should correctly import the necessary classes based on these file locations!
Return ```python your_code_here ``` with NO other texts,
your code:
"""


class RewriteUnitTest(Action):
    name: str = "RewriteUnitTest"

    async def write_code(self, prompt):
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

    async def run(self, msg):
        content= eval(msg.content)

        test_file_name = content['file_name']
        error_message = content['instruction']
        test_code = read_file(WORKSPACE_ROOT, test_file_name)
        prompt = PROMPT_TEMPLATE.format(
                test_code=test_code,
                error_message=error_message,
                )

        test_code = await self.write_code(prompt)
        await self._save(test_file_name, test_code)
        file_name = test_file_name.replace('test_', '')
        self._logger.fatal(test_file_name)
        result = {
            'file_name':      file_name,
            'test_file_name': test_file_name,
            'command':        ["python", f"{test_file_name}"]
            }
        return str(result)
