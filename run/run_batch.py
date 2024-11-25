import subprocess
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed

task_keys = [
    "exploration",
    "crossing",
    "flocking",
    "shaping",
    "bridging",
    "circling",
    "encircling",
    "covering",
    "clustering",
    "pursuing",
]

MAX_THREADS = 1  # Set the maximum number of threads you want to run concurrently


def run_batch(batch_num, task_name):
    print(f"Running batch {batch_num} for task {task_name}...")
    result = subprocess.run(
        [
            "python",
            "run_code.py",
            "--exp_batch",
            str(batch_num),
            "--task_name",
            task_name,
        ]
    )
    if result.returncode != 0:
        print(f"Batch {batch_num} encountered an error.")
    return result.returncode


def run_batches(task_name):
    batch_numbers = range(1, 51)  # Adjust range as needed
    with ThreadPoolExecutor(max_workers=MAX_THREADS) as executor:
        # Submit all batches to the executor and create progress bar
        future_to_batch = {
            executor.submit(run_batch, batch_num, task_name): batch_num
            for batch_num in batch_numbers
        }
        for future in tqdm(
            as_completed(future_to_batch),
            total=len(batch_numbers),
            desc=f"Running {task_name} batches",
        ):
            batch_num = future_to_batch[future]
            try:
                return_code = future.result()
                if return_code != 0:
                    print(f"Batch {batch_num} encountered an error and stopped.")
                    break
            except Exception as exc:
                print(f"Batch {batch_num} generated an exception: {exc}")


def run_tasks(task_name=None):
    if task_name:
        run_batches(task_name)
    else:
        for task_name in task_keys:
            print(f"\nRunning task {task_name}...")
            run_batches(task_name)


if __name__ == "__main__":
    run_tasks()
