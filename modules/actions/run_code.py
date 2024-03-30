import asyncio
import os
import sys
import traceback

from modules.actions.action import Action


class RunCode(Action):
    name: str = 'RunCode'

    async def _run_script(self, working_directory, command=[], print_output=True) -> str:
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

            if 'WARNING: cannot load logging configuration file, logging is disabled\n' in stderr_chunks:
                stderr_chunks.remove('WARNING: cannot load logging configuration file, logging is disabled\n')
            if stderr_chunks:
                return '\n'.join(stderr_chunks)
            else:
                return 'NONE'

        except asyncio.TimeoutError:
            self._context.log.format_message(content="Timeout", style="error")
            process.kill()
            return "Timeout"
        except Exception as e:
            self._context.log.format_message(content=f"error in run code: {e}", style="error")
            process.kill()

    async def _run(self, code_info, mode="script", **kwargs) -> str:
        command = code_info["command"]
        print(f"running command: {command}")
        if mode == "script":
            from modules.utils.root import root_manager

            result = await self._run_script(working_directory=root_manager.workspace_root, command=command)
            return result

    def process_response(self, response: str, **kwargs) -> str:
        return response
