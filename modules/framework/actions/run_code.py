import argparse
import asyncio
import json
import os
import pickle
import sys
import traceback

import rospy

from modules.file import logger
from modules.framework.action import ActionNode
from modules.framework.code_error import Bug
from modules.framework.context import WorkflowContext
from modules.utils import root_manager, get_project_root, run_script


class RunAllocateRun(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        self.call_times = 0
        super().__init__(next_text, node_name)

    async def _run(self):
        self.context.scoop = "global"
        self.call_times += 1
        if len(self.context.global_skill_tree.layers) == 0:
            logger.log(content="No task to run", level="error")
            return "No task to run"

        command = ["python", "allocate_run.py"]
        result = await run_script(
            working_directory=root_manager.workspace_root, command=command, timeout=5
        )
        print(result)
        return self._process_response(result)

    def _process_response(self, result: str):
        if result == "NONE":
            logger.log(content="Run allocate success", level="success")
            return result
        else:
            logger.log(content=f"Run allocate failed, result: {result}", level="error")
            return Bug(error_msg=result, error_code='\n\n'.join(self.context.global_skill_tree.functions_body),
                       error_function='')


class RunCode(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)

        self.start_id = None
        self.end_id = None
        self.task = None

    def _build_prompt(self):
        pass

    def setup(self, start: int, end: int):
        self.start_id = start
        self.end_id = end

    async def run(self, auto_next: bool = True) -> str:
        script = self.context.args.script
        command = ["python", script, str(self.start_id), str(self.end_id)]
        # print(f"运行命令: {command}")

        # 运行主脚本，并传递额外的输入参数
        result = await run_script(
            working_directory=root_manager.workspace_root, command=command, timeout=self.context.args.timeout
        )
        return result

    def _process_response(self, response: str) -> str:
        return response


class RunCodeAsync(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = "", env=None):
        self.call_times = 0
        super().__init__(next_text, node_name)

    async def _run(self):
        self.call_times += 1
        self.context.scoop = "local"
        start_idx = rospy.get_param("robot_start_index")
        end_idx = rospy.get_param("robot_end_index")
        total_robots = end_idx - start_idx + 1
        num_processes = min(3, total_robots)  # 并行进程数
        robots_per_process = total_robots // num_processes

        robot_ids = list(range(start_idx, end_idx + 1))
        robot_id_chunks = [robot_ids[i:i + robots_per_process] for i in range(0, total_robots, robots_per_process)]
        tasks = []
        result_list = []
        try:
            for chunk in robot_id_chunks:
                action = RunCode()
                action.setup(chunk[0], chunk[-1])
                task = asyncio.create_task(action.run())
                tasks.append(task)
            result_list = list(set(await asyncio.gather(*tasks)))
        except Exception as e:
            print("Error in RunCodeAsync: ", e)
            traceback.print_exc()
        finally:
            os.system("pgrep -f run.py | xargs kill -9")
            return self._process_response(result_list)

    def _process_response(self, result: list):
        if ("NONE" in result or "Timeout" in result) and len(result) == 1:
            logger.log(content="Run code success", level="success")
            return "NONE"
        logger.log(content=f"Run code failed, result: {result}", level="error")
        result_content = "\n".join(result)
        if self.call_times < 3:
            return Bug(error_msg=result_content, error_function='',
                       error_code='/n/n'.join(self.context.local_skill_tree.functions_body))
        else:
            logger.log(content="Run code failed 3 times, stop running", level="error")


def init_workflow(args, env=None) -> ActionNode:
    from modules.framework.handler import BugLevelHandler
    from modules.framework.handler import FeedbackHandler
    from modules.framework.actions import DebugError, CodeImprove, VideoCriticize
    context = WorkflowContext()
    debug_code = DebugError()
    code_improver = CodeImprove("feedback")
    run_allocate = RunAllocateRun("run allocate")
    run_code = RunCodeAsync("run code", env=env)
    video_critic = VideoCriticize("")
    run_allocate._next = run_code
    # initialize error handlers

    bug_handler = BugLevelHandler()
    bug_handler.next_action = debug_code
    debug_code._next = run_allocate

    hf_handler = FeedbackHandler()
    hf_handler.next_action = code_improver
    code_improver._next = run_allocate

    # link error handlers
    chain_of_handler = bug_handler
    bug_handler.successor = hf_handler

    if args.feedback != 'None':
        run_allocate.error_handler = chain_of_handler
        run_code.error_handler = chain_of_handler

        run_code._next = video_critic
        video_critic.error_handler = chain_of_handler
    if args.target_pkl != 'None':
        path = f'{get_project_root()}/workspace/{args.experiment_path}'
        context.load_from_file(path + "/" + args.target_pkl)
        context.args = args

    return run_allocate


def runcode(timeout=20, feedback="VLM", experiment_path="clustering/2024-10-21_03-04-33",
            target_pkl="WriteRun.pkl", script="run.py", human_feedback=False, env=None):
    """
    Run the simulation with custom parameters (synchronously).

    Args:
        timeout (int): Total time for the simulation.
        feedback (str): Optional feedback type ("human", "VLM", "None").
        experiment_path (str): Data path for the simulation.
        target_pkl (str): Path to the target pkl file.
        script (str): Script to run for the simulation.
        human_feedback (bool): Whether to use human feedback.
    """

    parser = argparse.ArgumentParser(description="Run simulation with custom parameters.")

    parser.add_argument("--timeout", type=int, default=20, help="Total time for the simulation")
    parser.add_argument("--feedback", type=str, default="VLM", help="Optional: human, VLM, None, Result feedback")
    parser.add_argument("--experiment_path", type=str, default='clustering/2024-10-21_03-04-33',
                        help="Data path for the simulation")
    parser.add_argument("--target_pkl", type=str, default="WriteRun.pkl", help="Data path for the simulation")
    parser.add_argument("--script", type=str, default="run.py", help="Script to run")
    parser.add_argument("--human_feedback", type=bool, default=False, help="Whether to use human feedback")

    args = parser.parse_args(args=[
        f"--timeout={timeout}",
        f"--feedback={feedback}",
        f"--experiment_path={experiment_path}",
        f"--target_pkl={target_pkl}",
        f"--script={script}",
        f"--human_feedback={str(human_feedback)}"
    ])

    path = f'{get_project_root()}/workspace/{args.experiment_path}'

    rospy.set_param("path", args.experiment_path)
    root_manager.update_root(path)

    first_action = init_workflow(args, env=env)

    # 使用 asyncio.run() 来运行异步的 first_action.run()
    asyncio.run(first_action.run())

    # 保存执行结果
    first_action.context.save_to_file(f"{path}/run_code.pkl")


if __name__ == "__main__":
    runcode()
