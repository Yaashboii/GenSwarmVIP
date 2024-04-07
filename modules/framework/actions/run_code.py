import asyncio
import os
from os import listdir
import sys

from modules.framework.action import ActionNode
from modules.utils.root import root_manager
from modules.utils import get_param, call_reset_environment
from modules.framework.code_error import Bug


class RunCode(ActionNode):
    def __init__(self, next_text: str = '', node_name: str = ''):
        super().__init__(next_text, node_name)
        self._id = None

    def _build_prompt(self):
        pass

    def setup(self, run_id: int):
        self._id = run_id

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
            self._context.logger.log(content="Timeout", level="error")
            process.kill()
            return "Timeout"
        except Exception as e:
            self._context.logger.log(content=f"error in run code: {e}", level="error")
            process.kill()

    async def _run(self) -> str:
        command = ["python", "run.py", str(self._id)]
        print(f"running command: {command}")

        result = await self._run_script(working_directory=root_manager.workspace_root, command=command)
        return result

    def _process_response(self, response: str) -> str:
        return response


class RunCodeAsync(ActionNode):
    async def _run(self):
        robot_num = get_param('robots_num')
        tasks = []

        try:
            self._context.logger.log(content="call reset environment: start")
            call_reset_environment(True)
            for robot_id in range(robot_num):
                action = RunCode()
                action.setup(robot_id)
                task = asyncio.create_task(action.run())
                tasks.append(task)
            result_list = list(set(await asyncio.gather(*tasks)))
        finally:
            self._context.logger.log(content="call reset environment: end")
            call_reset_environment(True)
            from modules.utils import generate_video_from_frames, root_manager
            data_root = root_manager.data_root
            number = len(listdir(f"{data_root}/frames")) - 1
            generate_video_from_frames(
                frames_folder=f"{data_root}/frames/frame{number}",
                video_path=f"{data_root}/output{number}.mp4",
            )
            self._context.logger.log(f"synthesize frame{number} ---> output{number}.mp4")
        return result_list

    def _process_response(self, result: str):
        if 'NONE' in result and len(result) == 1:
            return "ALL_PASS"
        elif 'Timeout' in result and len(result) == 1:
            return "ALL_PASS"
        result_content = '\n'.join(result)
        return Bug(result_content)


if __name__ == "__main__":
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/2024-04-07_15-09-48'
    root_manager.update_root(path, set_data_path=False)

    run_code = RunCodeAsync('run code')
    # run_code.context.load_from_file(path + "/write_run.pkl")
    asyncio.run(run_code.run())

    run_code.context.save_to_file(f'{path}/run_code.pkl')
