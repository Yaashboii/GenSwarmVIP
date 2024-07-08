import asyncio
import os
from os import listdir
import sys

import rospy

from modules.framework.action import ActionNode
from modules.utils.root import root_manager
from modules.utils.common import get_param, call_reset_environment
from modules.framework.code_error import Bug, Feedback
from modules.file.log_file import logger
from modules.framework.code.function_tree import FunctionTree


class RunCode(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self.start_id = None
        self.end_id = None

    def _build_prompt(self):
        pass

    def setup(self, start: int, end: int):
        self.start_id = start
        self.end_id = end

    async def _run_script(
            self, working_directory, command=[], print_output=True
    ) -> str:
        working_directory = str(working_directory)
        env = os.environ.copy()

        process = await asyncio.create_subprocess_exec(
            *command,
            cwd=working_directory,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )

        stdout_chunks, stderr_chunks = [], []

        async def read_stream(stream, accumulate, is_stdout=True):
            while True:
                line_bytes = await stream.readline()
                if not line_bytes:
                    break
                line = line_bytes.decode("utf-8")
                accumulate.append(line)
                if print_output:
                    print(
                        line,
                        end="" if is_stdout else "",
                        file=sys.stderr if not is_stdout else None,
                    )

        try:
            # Apply timeout to the gather call using asyncio.wait_for
            if hasattr(self.context.args, "timeout"):
                timeout = self.context.args.timeout
            else:
                timeout = 30
            await asyncio.wait_for(
                asyncio.gather(
                    read_stream(process.stdout, stdout_chunks, is_stdout=True),
                    read_stream(process.stderr, stderr_chunks, is_stdout=False),
                ),
                timeout=timeout,
            )

            if (
                    "WARNING: cannot load logging configuration file, logging is disabled\n"
                    in stderr_chunks
            ):
                stderr_chunks.remove(
                    "WARNING: cannot load logging configuration file, logging is disabled\n"
                )
            if stderr_chunks:
                return "\n".join(stderr_chunks)
            else:
                return "NONE"

        except asyncio.TimeoutError:
            logger.log(content="Timeout", level="error")
            process.kill()
            await process.wait()  # Ensure the process is terminated
            return "Timeout"
        except asyncio.CancelledError:
            logger.log(content="Cancelled", level="error")
            process.kill()
            await process.wait()  # Ensure the process is terminated
            raise
        except Exception as e:
            logger.log(content=f"error in run code: {e}", level="error")
            process.kill()
            await process.wait()  # Ensure the process is terminated
            return f"error in run code: {e}"
        finally:
            # Ensure the process is terminated in case of any other unexpected errors
            if process.returncode is None:  # Check if process is still running
                process.kill()
                await process.wait()

    async def run(self) -> str:
        command = ["python", "run.py", str(self.start_id), str(self.end_id)]
        # print(f"running command: {command}")

        result = await self._run_script(
            working_directory=root_manager.workspace_root, command=command
        )
        return result

    def _process_response(self, response: str) -> str:
        return response


class RunCodeAsync(ActionNode):
    async def _run(self):
        from modules.utils.media import generate_video_from_frames

        start_idx = rospy.get_param("robot_start_index")
        end_idx = rospy.get_param("robot_end_index")
        total_robots = end_idx - start_idx + 1

        num_processes = min(10, total_robots)  # Number of processes
        robots_per_process = total_robots // num_processes

        robot_ids = list(range(start_idx, end_idx + 1))
        robot_id_chunks = [robot_ids[i:i + robots_per_process] for i in range(0, total_robots, robots_per_process)]
        tasks = []
        result_list = []
        try:
            logger.log(content="call reset environment: start")
            # call_reset_environment(False)
            for chunk in robot_id_chunks:
                action = RunCode()
                action.setup(chunk[0], chunk[-1])
                task = asyncio.create_task(action.run())
                tasks.append(task)
            result_list = list(set(await asyncio.gather(*tasks)))
        finally:
            logger.log(content="call reset environment: end")
            # call_reset_environment(False)
            data_root = root_manager.data_root
            number = len(listdir(f"{data_root}/frames")) - 1
            generate_video_from_frames(
                frames_folder=f"{data_root}/frames/frame{number}",
                video_path=f"{data_root}/output{number}.mp4",
            )
            logger.log(f"synthesize frame{number} ---> output{number}.mp4")
            return self._process_response(result_list)

    def _process_response(self, result: list):
        if ("NONE" in result or "Timeout" in result) and len(result) == 1:
            logger.log(content="Run code success", level="success")
            return "NONE"
        logger.log(content=f"Run code failed,result{result}", level="error")
        result_content = "\n".join(result)
        return Bug(error_msg=result_content, error_function='')


if __name__ == "__main__":
    from modules.utils import root_manager
    import asyncio
    from modules.framework.handler import BugLevelHandler
    from modules.framework.handler import FeedbackHandler
    from modules.framework.actions.video_criticize import VideoCriticize

    from modules.framework.actions import *
    import argparse

    # data = '2024-06-19_10-50-33'
    # data = '2024-06-21_09-55-56'
    data = 'sequential/flocking/2024-07-04_10-14-33'
    path = f"../../../workspace/{data}"
    rospy.set_param("path", data)
    root_manager.update_root(path)
    debug_code = DebugError("fixed code")
    human_feedback = Criticize("feedback")
    run_code = RunCodeAsync("run code")
    video_critic = VideoCriticize("")

    # initialize error handlers
    bug_handler = BugLevelHandler()
    bug_handler.next_action = debug_code
    debug_code._next = run_code
    # critic_handler = CriticLevelHandler()
    hf_handler = FeedbackHandler()
    hf_handler.next_action = human_feedback
    human_feedback._next = run_code
    # link error handlers
    chain_of_handler = bug_handler
    bug_handler.successor = hf_handler
    run_code.error_handler = chain_of_handler
    run_code._next = video_critic
    video_critic.error_handler = chain_of_handler
    parser = argparse.ArgumentParser(
        description="Run simulation with custom parameters."
    )

    parser.add_argument(
        "--timeout", type=int, default=40, help="Total time for the simulation"
    )

    args = parser.parse_args()
    run_code.context.load_from_file(path + "/WriteRun.pkl")
    run_code.context.args = args
    asyncio.run(run_code.run())
    run_code.context.save_to_file(f"{path}/run_code.pkl")
