import asyncio
import os
import sys
import traceback
from typing import Tuple

from modules.framework.action import ActionNode
from modules.prompt.run_code_prompt import PROMPT_TEMPLATE, CONTEXT


class RunCode(ActionNode):
    @classmethod
    def _run_text(cls, code) -> Tuple[str, str]:
        try:
            # We will document_store the result in this dictionary
            namespace = {}
            exec(code, namespace)
            return namespace.get("result", ""), ""
        except Exception:
            # If there is an error in the code, return the error message
            return "", traceback.format_exc()

    async def _run_script(self, working_directory, command=[], print_output=True) -> Tuple[str, str]:
        working_directory = str(working_directory)
        env = os.environ.copy()

        process = await asyncio.create_subprocess_exec(
            *command,
            cwd=working_directory,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env
        )

        stdout_chunks, stderr_chunks = [], []

        # Simplified stream reading with direct print control
        async def read_stream(stream, accumulate, is_stdout=True):
            while True:
                line_bytes = await stream.readline()
                if not line_bytes:
                    break
                line = line_bytes.decode('utf-8')
                accumulate.append(line)
                if print_output:
                    print(line, end='' if is_stdout else '', file=sys.stderr if not is_stdout else None)

        try:
            # Apply timeout to the gather call using asyncio.wait_for
            if hasattr(self._context.args, 'timeout'):
                timeout = self._context.args.timeout
            else:
                timeout = 30
            await asyncio.wait_for(
                asyncio.gather(
                    read_stream(process.stdout, stdout_chunks, is_stdout=True),
                    read_stream(process.stderr, stderr_chunks, is_stdout=False)
                ),
                timeout=timeout
            )

        except asyncio.TimeoutError:
            self._logger.info("The command did not complete within the given timeout.")
            process.kill()
            stdout, stderr = await process.communicate()
            return stdout.decode('utf-8'), 'The command did not complete within the given timeout: ' + stderr.decode(
                'utf-8')
        except Exception as e:
            self._logger.error(f"An error occurred while running the command: {e}")
            process.kill()
            stdout, stderr = await process.communicate()
            return stdout.decode('utf-8'), f"An error occurred while running the command: {e}"

        # Join collected lines into single strings
        stdout = ''.join(stdout_chunks)
        stderr = ''.join(stderr_chunks)
        return stdout, stderr

    async def _run(self, code_info, mode="script", **kwargs) -> str:
        command = code_info["command"]
        self._logger.info(f"Running {' '.join(command)}")
        outs, errs = "", ""
        if mode == "script":
            # Note: must call call_reset_environment before and after running the script
            from modules.utils.root import root_manager

            outs, errs = await self._run_script(working_directory=root_manager.workspace_root, command=command)

        self._logger.info(f"Outs: {outs}")
        self._logger.error(f"Errs: {errs}")

        return str(outs + errs)

    def _process_response(self, response: str) -> str:
        return response