import argparse
import asyncio
import threading

from modules.framework.workflow import Workflow
from modules.utils import root_manager


def main(task: str, args: argparse.Namespace):
    init_workspace()
    workflow = Workflow(task)
    asyncio.run(workflow.run(args))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run simulation with custom parameters.")
    # TODO: add more arguments and use them in the simulation
    parser.add_argument("--simulation_time", type=int, default=60, help="Total time for the simulation")
    parser.add_argument("--robot_num", type=int, default=3, help="Number of robots in the simulation")
    parser.add_argument("--if_leader", type=bool, default=True, help="If create a leader")
    parser.add_argument("--if_render", type=bool, default=True, help="If render the frames")

    args = parser.parse_args()

    task_list = [
        {
            "task": "Forming flocking with other robots.",
            "run_times": 1
        },

    ]
    # TODO: fix bug when run multiple tasks and multiple times
    for task in task_list:
        content = task["task"]
        times = task["run_times"]
        for _ in range(times):
            main(content, args)
