import subprocess
import time
from concurrent.futures import ThreadPoolExecutor


def run_command(command):
    print(f"Starting run for command: {command}")
    process = subprocess.Popen(command, shell=True)
    process.wait()


def run_multiple_times(command, num_times, max_workers=1):
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        for _ in range(num_times):
            executor.submit(run_command, command)


if __name__ == "__main__":
    command_to_run = "python run.py"
    # 2024-06-11_20-53-18
    num_runs = 5
    max_workers = 1

    run_multiple_times(command_to_run, num_runs, max_workers)
