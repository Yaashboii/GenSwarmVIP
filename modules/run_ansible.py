import os
import subprocess


if __name__ == "__main__":
    # 获取当前工作目录
    current_directory = os.path.dirname(os.path.abspath(__file__))

    # 构造 docker-compose.yml 的路径
    compose_file_path = os.path.join(current_directory, "../docker/docker-compose.yml")

    # 获取绝对路径
    absolute_path = os.path.abspath(compose_file_path)

    # 设置环境变量
    os.environ["MY_VARIABLE"] = "123"
    os.environ["STAGE"] = "1"

    # 调用 docker-compose 命令并显示输出
    result = subprocess.run(
        ["docker-compose", "-f", compose_file_path, "up", "deploy"],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    # 打印标准输出和标准错误
    print(result.stdout)
    print(result.stderr)
    print(compose_file_path)
