from modules.actions.action import Action
from modules.utils import parse_code

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

    async def write_code(self, prompt):
        code_rsp = await self._aask(prompt)

        try:
            code = parse_code(text=code_rsp)
        except Exception:
            # Handle the exception if needed
            self._logger.error(f"Can't parse the code: {code_rsp}")

            # Return code_rsp in case of an exception, assuming llm just returns code as it is and doesn't wrap it inside ```
            code = code_rsp
        return code

    async def run(self, code, error_message):
        prompt = PROMPT_TEMPLATE.format(
                code=code,
                error_message=error_message,
                )
        code = await self.write_code(prompt)
        return code
