from typing import List, Any
from typing import List, Any

from modules.stages.stage import Stage
from modules.actions import ActionResult, WriteUnitTest, RunCode
from modules.utils import read_file, check_file_exists
from const import WORKSPACE_ROOT


PROMPT_TEMPLATE = """
NOTICE
1. Role: You are a QA engineer; the main goal is to design, develop, and execute PEP8 compliant, well-structured, maintainable test cases and scripts for Python 3.9. Your focus should be on ensuring the product quality of the entire project through systematic testing.
2. Requirement: Based on the context, please develop complete, robust, and reusable unit test cases.
3. Attention1: If there are any settings in your tests, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE.
4. Think before writing: What should be tested and validated in this document? What edge cases could exist? What might fail?
5. CAREFULLY CHECK THAT YOU DON'T MISS ANY NECESSARY TEST CASES/SCRIPTS IN THIS FILE.
-----
## Given the following code, please write appropriate test cases using Python's unittest framework to verify the correctness and robustness of this code:
```python
{code_to_test}
```
The test file is at the same directory level as the source file (filename=core.py).Use from core import ... and from env import env to import the source file.

you should correctly import the necessary classes based on these file locations!
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


class TestStage(Stage):
    def __init__(self, action: WriteUnitTest, filename):
        super(TestStage, self).__init__()
        self._action = action
        self._run_code = RunCode()
        self._input = ""
        self._filename = filename
        self._test_filename = "test_" + filename

    def update(self, data: str):
        self._input = data

    def _run(self) -> (str, List[Any]):
        if not check_file_exists(WORKSPACE_ROOT, self._test_filename):
            prompt = PROMPT_TEMPLATE.format(
            code_to_test=self._input)
            self._action.prompt_template = prompt
            command = self._action.run(ActionResult(0, prompt))
        else:
            test_code = read_file(WORKSPACE_ROOT, self._test_filename)
            prompt = REWRITE_PROMPT_TEMPLATE.format(
                test_code=test_code,
                error_message=self._input)
            command = self._action.run(ActionResult(0, prompt))

        code_info = {
            "command": command,
            "file_name": self._filename,
            "test_file_name": self._test_filename
        }
        error, keys = self._run_code.run(ActionResult(0, str(code_info)))
        return (error, keys)
