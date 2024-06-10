import argparse
import asyncio
import threading

from modules.framework.workflow import Workflow
from modules.utils import root_manager


async def run_task(task: str, args: argparse.Namespace):
    workflow = Workflow(task, args=args)
    await workflow.run()


async def run_all_tasks_sequentially(task_list, args):
    for task in task_list:
        content = task["task"]
        times = task["run_times"]
        for i in range(times):
            # root_manager.update_root()
            await run_task(content, args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run simulation with custom parameters."
    )
    # TODO: add more arguments and use them in the simulation
    parser.add_argument(
        "--timeout", type=int, default=30, help="Total time for the simulation"
    )
    parser.add_argument(
        "--robot_num", type=int, default=3, help="Number of robots in the simulation"
    )
    parser.add_argument(
        "--if_leader", type=bool, default=True, help="If create a leader"
    )
    parser.add_argument(
        "--if_render", type=bool, default=True, help="If render the frames"
    )

    args = parser.parse_args()

    task_list = [
        # {
        #     "task": 'form a circle with other robots.'
        #             'Additionally, avoid collisions with obstacles and boundaries.'
        #             'Keep a distance of 0.5 meters from obstacles.'
        #             'Keep a distance of 0.5 meters from boundaries.',
        #     "run_times": 1
        # },
        {
            "task": "Integrate into a flock, adhering to cohesion by staying connected, alignment by moving together, and separation by maintaining at least 50 meters between robots."
                    "Additionally, avoid collisions with obstacles and boundaries."
                    "Keep a distance of 50 meters from obstacles."
                    "Keep a distance of 50 meters from boundaries.",
            "run_times": 1,
        },
        # {
        #     "task": "Coordinate with other robots within the sensing range to encircle the prey, leaving no gaps to prevent its escape."
        #             "Additionally, avoid collisions with obstacles"
        #             "avoid collisions with other robots",
        #     "run_times": 1,
        # },
        # {
        #     "task": "Coordinate with other robots within the sensing range to pursuit the prey, leaving no gaps to prevent its escape."
        #             "Additionally, Keep a distance of 0.5 meters from obstacles"
        #             "As quickly as possible,Keep a distance of 0.5 meters from robots",
        #     "run_times": 1,
        # },
        # {
        #     "task": "The robot needs to move to the target position while maintaining a distance of at least 1 meter from other robots to avoid collisions.",
        #     "run_times": 1,
        # },
        # {
        #     "task": "You need to form an equilateral triangle with a side length of 2 with other robots, and the robots should be evenly distributed along the triangle's outline."
        #             'Additionally, avoid collisions with obstacles and boundaries.'
        #             'Keep a distance of 0.5 meters from obstacles.'
        #             'Keep a distance of 0.5 meters from boundaries.',
        #     "run_times": 1,
        # },
        # {
        #     "task": "You need to form a square with a side length of 2 with other robots, and the robots should be evenly distributed along the square's outline."
        #             'Additionally, avoid collisions with obstacles and boundaries.'
        #             'Keep a distance of 0.5 meters from obstacles.'
        #             'Keep a distance of 0.5 meters from boundaries.',
        #     "run_times": 1,
        # },

    ]
    asyncio.run(run_all_tasks_sequentially(task_list, args))
