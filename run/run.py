from parser import ParameterService
from run_single import run_task
import subprocess
from concurrent.futures import ThreadPoolExecutor


def run_command(command):
    print(f"Starting run for command: {command}")
    process = subprocess.Popen(command, shell=True)
    process.wait()


def run_multiple_times(command, num_times, max_workers=1):
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        for _ in range(num_times):
            executor.submit(run_command, command)


def run_experiment(args):
    pass


if __name__ == "__main__":
    parameter_service = ParameterService()
    config_file = 'base_config.yaml'
    parameter_service.add_arguments_from_yaml(f'../config/experiment_config/{config_file}')
    args = parameter_service.args
    print(parameter_service.format_arguments_as_table(args))
    if args.run_experiment_name:
        run_command(f'python run_single.py ../config/experiment_config/{config_file}')
