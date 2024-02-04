from typing import List, Any

from const import ENV_CODE, WORKSPACE_ROOT
from modules.stages.stage import Stage
from modules.actions import ActionResult, WriteCode
from modules.utils import read_file, check_file_exists


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

REWRITE_PROMPT_TEMPLATE = """
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

class WriteCoreStage(Stage):
    def __init__(self,  action: WriteCode=None):
        super().__init__()
        self._input = ""
        self._action = action

    def update(self, data: str):
        self._input = data

    def _run(self) -> (str, List[Any]):
        if not check_file_exists(WORKSPACE_ROOT, "core.py"):
            prompt = PROMPT_TEMPLATE.format(
                instruction=self._input,
                code=ENV_CODE)
        else:
            code = read_file(WORKSPACE_ROOT, "core.py")
            prompt = REWRITE_PROMPT_TEMPLATE.format(
                code=code,
                error_message=self._input)
        res = self._action.run(ActionResult(0, prompt))
        return (res.message, [])