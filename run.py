import argparse
import asyncio
import threading

from modules.framework.workflow import Workflow
from modules.env.environment import Env
from modules.utils import init_workspace


def main(task: str):
    init_workspace()
    workflow = Workflow(task)
    asyncio.run(workflow.run())


def run_env():
    env = Env(if_leader=True, n_robots=100, size=(10, 10))
    env.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run simulation with custom parameters.")
    # TODO: add more arguments and use them in the simulation
    parser.add_argument("--simulation_time", type=int, default=60, help="Total time for the simulation")

    args = parser.parse_args()

    task_list = [
        "Gather these robots together at the position of leader robot",
        "form a circle with the robots at the position of leader robot",
    ]
    # TODO: fix bug when run multiple tasks
    # for task in task_list:
    main(task_list[0])
