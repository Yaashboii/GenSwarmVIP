import subprocess
from tqdm import tqdm


def run_batches():
    # 遍历 1 到 10，使用 tqdm 创建进度条
    for batch_num in tqdm(range(1, 11), desc="Running batches", leave=True):
        print(f"\nRunning batch {batch_num}...")

        # 使用 subprocess.run 来运行脚本，等待每个脚本执行结束再继续下一个
        result = subprocess.run(
            ["python", "run_code.py", "--exp_batch", str(batch_num)]
        )

        # 检查返回值，如果出错（返回码不为零），则打印错误并中止循环
        if result.returncode != 0:
            print(f"Batch {batch_num} encountered an error and stopped.")
            break


if __name__ == "__main__":
    run_batches()
