import asyncio
import argparse

from modules.file import logger
from modules.framework.workflow import Workflow
from modules.prompt import get_user_commands


async def run_task(task: str, args: argparse.Namespace):
    workflow = Workflow(task, args=args)
    await workflow.run()


if __name__ == '__main__':
    from modules.utils import root_manager
    from parser import ParameterService

    parameter_service = ParameterService()
    config_file = 'experiment_config.yaml'
    parameter_service.add_arguments_from_yaml(f'../config/{config_file}')

    experiment_name = parameter_service.args.run_experiment_name[0]
    task = get_user_commands(experiment_name)[0]

    args = parameter_service.args
    root_manager.update_root(args=args)
    logger.log(f'\n{parameter_service.format_arguments_as_table(args)}', 'warning')

    asyncio.run(run_task(task, parameter_service.args))
