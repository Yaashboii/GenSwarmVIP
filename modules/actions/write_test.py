from modules.actions.action import Action
from modules.utils import parse_code

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


class WriteUnitTest(Action):
    name: str = "WriteUnitTest"

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

    async def run(self, code_to_test):
        prompt = PROMPT_TEMPLATE.format(
                code_to_test=code_to_test,
                )
        code = await self._write_code(prompt)
        return code
