import os
import re
import subprocess
import traceback
from typing import Tuple
from modules.actions.action import Action
from loguru import logger

from const import WORKSPACE_ROOT
from modules.utils import read_file
from modules.framework.message import Message

PROMPT_TEMPLATE = """
Role: You are a senior development and qa engineer, your role is summarize the code running result.
If the running result does not include an error, you should explicitly approve the result.
On the other hand, if the running result indicates some error, you should point out which part, the development code or the test code, produces the error,
and give specific instructions on fixing the errors. Here is the code info:
{context}
Now you should begin your analysis
---
## Instruction:
Please summarize the cause of the errors and give correction instruction
## File To Rewrite: Determine the ONE file to rewrite in order to fix the error, for example, xyz.py, or test_xyz.py
## Status:
Determine if all of the code works fine, if so write PASS, else FAIL,
WRITE ONLY ONE WORD, PASS OR FAIL, IN THIS SECTION
## Send To:
Please write Engineer if the errors are due to problematic development codes, and QaEngineer to problematic test codes, and NoOne if there are no errors,
WRITE ONLY ONE WORD, Engineer OR QaEngineer OR NoOne, IN THIS SECTION.
---
---
You should fill in necessary Instruction, status, and finally return all content between the --- segment line.
"""

CONTEXT = """
## Development Code File Name
{code_file_name}
## Development Code
```python
{code}
```
## Test File Name
{test_file_name}
## Test Code
```python
{test_code}
```
## Running Command
{command}
## Running Output
standard output: {outs};
standard errors: {errs};
"""


class RunCode(Action):
    name: str = 'RunCode'

    @classmethod
    async def _run_text(cls, code) -> Tuple[str, str]:
        try:
            # We will document_store the result in this dictionary
            namespace = {}
            exec(code, namespace)
            return namespace.get("result", ""), ""
        except Exception:
            # If there is an error in the code, return the error message
            return "", traceback.format_exc()

    @classmethod
    async def _run_script(cls, working_directory, command=[]) -> Tuple[str, str]:
        working_directory = str(working_directory)
        # Copy the current environment variables
        env = os.environ.copy()

        # Start the subprocess
        process = subprocess.Popen(
                command, cwd=working_directory, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env
                )

        try:
            # Wait for the process to complete, with a timeout
            stdout, stderr = process.communicate(timeout=10)
            return stdout.decode("utf-8"), stderr.decode("utf-8")
        except subprocess.TimeoutExpired:
            logger.info("The command did not complete within the given timeout.")
            process.kill()  # Kill the process if it times out
            stdout, stderr = '', 'The command did not complete within the given timeout.'
            return stdout, stderr

    async def run(self, code_info, mode="script", **kwargs) -> str:
        code_info = eval(code_info)
        command = code_info["command"]
        code_file_name = code_info["file_name"]
        code = read_file(directory=WORKSPACE_ROOT, filename=code_file_name)
        test_file_name = code_info["test_file_name"]
        test_code = read_file(directory=WORKSPACE_ROOT, filename=test_file_name)

        logger.info(f"Running {' '.join(command)}")
        if mode == "script":
            outs, errs = await self._run_script(working_directory=WORKSPACE_ROOT, command=command, **kwargs)
        elif mode == "text":
            outs, errs = await self._run_text(code=code)

        logger.info(f"{outs=}")
        logger.info(f"{errs=}")

        context = CONTEXT.format(
                code=code,
                code_file_name=code_file_name,
                test_code=test_code,
                test_file_name=test_file_name,
                command=" ".join(command),
                outs=outs[:500],  # outs might be long but they are not important, truncate them to avoid token overflow
                errs=errs[:10000],  # truncate errors to avoid token overflow
                )

        prompt = PROMPT_TEMPLATE.format(context=context)
        rsp = await self._ask(prompt)
        status = re.search("Status:\s*(.+)", rsp, re.IGNORECASE).group(1)
        if status == "PASS":
            if code_file_name == "run.py":
                send_to = "NoOne"
            else:
                send_to = "Engineer"
            instruction = ''
            file_name = ''
        else:
            send_to = re.search("Send To:\s*(.+)", rsp, re.IGNORECASE).group(1)
            instruction = re.search("Instruction:\s*(.+)", rsp, re.IGNORECASE).group(1)
            file_name = re.search("File To Rewrite:\s*(.+\\.py)", rsp, re.IGNORECASE).group(1)

        result = {
            "file_name":   file_name,
            "instruction": instruction,
            }

        msg = Message(
                content=str(result),
                role='Engineer',
                cause_by=RunCode,
                sent_from='Engineer',
                send_to={send_to},
                )

        return msg
