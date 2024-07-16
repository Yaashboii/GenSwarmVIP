import argparse
import sys

from modules.file import logger
from modules.framework.workflow import Workflow
from modules.prompt.user_requirements import get_user_commands


async def run_task(task: str, args: argparse.Namespace):
    workflow = Workflow(task, args=args)
    await workflow.run()


if __name__ == '__main__':
    from modules.utils import root_manager
    from parser import ParameterService
    import asyncio

    #
    # parameter_service = ParameterService()
    # file_path = sys.argv[1]
    # # parameter_service.remove_argument(file_path)
    # args = parameter_service.add_arguments_from_yaml(file_path)
    # args = parameter_service.args
    # run_task(task, args)
    parameter_service = ParameterService()
    config_file = 'experiment_config.yaml'
    parameter_service.add_arguments_from_yaml(f'../config/{config_file}')
    task = get_user_commands('flocking')[0]

    args = parameter_service.args
    root_manager.update_root(args=args)
    logger.log(f'\n{parameter_service.format_arguments_as_table(args)}', 'warning')

    asyncio.run(run_task(task, parameter_service.args))
