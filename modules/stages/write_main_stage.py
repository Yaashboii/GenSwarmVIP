from typing import List, Any

from modules.actions import ActionResult, WriteCode
from modules.stages.stage import Stage
from const import WORKSPACE_ROOT, ENV_CODE
from modules.utils import read_file, check_file_exists


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

REWRITE_PROMPT_TEMPLATE = """
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


core_code = read_file(directory=WORKSPACE_ROOT, filename='core.py')
user_requirements = read_file(directory=WORKSPACE_ROOT, filename='requirements.py')

class WriteMainStage(Stage):
    def __init__(self,  action: WriteCode=None):
        super().__init__()
        self._action = action
        self._input = ""

    def update(self, data: str):
        self._input = data

    def _run(self) -> (str, List[Any]):
        if not check_file_exists(WORKSPACE_ROOT, "run.py"):
            prompt = PROMPT_TEMPLATE.format(
                user_requirements=user_requirements,
                core_code=core_code,
                env_code=ENV_CODE)
        else:
            test_code = read_file(WORKSPACE_ROOT, "test_run.py")
            prompt = REWRITE_PROMPT_TEMPLATE.format(
                test_code=test_code,
                error_message=self._input)
        res = self._action.run(ActionResult(0, prompt))
        return (res.message, [])
