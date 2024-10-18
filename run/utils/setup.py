"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""

import os
import shutil


def setup_metagpt(directory):
    # 确保目录存在
    if not os.path.exists(directory):
        print(f"目录 {directory} 不存在")
        return
    # 如果目录下没有py文件，遍历所以的文件夹找到有py文件的，将其内部的所以py文件复制到directory下
    has_py_files = any(file.endswith(".py") for file in os.listdir(directory))
    if not has_py_files:
        for root, _, files in os.walk(directory):
            for file in files:
                if file.endswith(".py"):
                    src_file = os.path.join(root, file)
                    dest_file = os.path.join(directory, file)
                    shutil.copy(src_file, dest_file)
                    print(f"复制文件 {src_file} 到 {dest_file}")
    # 将apis.py文件复制到directory下
    source_file = os.path.join(
        "../modules/deployment/execution_scripts", "apis_meta.py"
    )
    if os.path.exists(source_file):
        shutil.copy(source_file, directory)
    else:
        print(f"文件 {source_file} 不存在")
        return
    source_file = os.path.join("../modules/deployment/execution_scripts", "run_meta.py")
    if os.path.exists(source_file):
        shutil.copy(source_file, directory)
    else:
        print(f"文件 {source_file} 不存在")
        return

    # 遍历文件夹下所有python文件，开头加上from apis import *
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".py") and file != "apis_meta.py":
                file_path = os.path.join(root, file)
                with open(file_path, "r+", encoding="utf-8") as f:
                    content = f.read()
                    if not content.startswith("from apis_meta import *"):
                        f.seek(0, 0)
                        f.write("from apis_meta import *\n" + content)

    # 查找是否存在time.sleep(xx),或者rate.sleep() 如果有将其注释
    import re

    sleep_patterns = [
        re.compile(r"(\s*)time\.sleep\(\s*.*?\s*\)"),
        re.compile(r"(\s*)rate\.sleep\(\s*.*?\s*\)"),
    ]

    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".py"):
                file_path = os.path.join(root, file)
                with open(file_path, "r", encoding="utf-8") as f:
                    lines = f.readlines()

                with open(file_path, "w", encoding="utf-8") as f:
                    for line in lines:
                        for pattern in sleep_patterns:
                            match = pattern.match(line)
                            if match:
                                line = f"{match.group(1)}# {line}"
                                break
                        f.write(line)


def setup_cap(directory):
    source_file = os.path.join(
        "../modules/deployment/execution_scripts", "apis_meta.py"
    )

    if os.path.exists(source_file):
        shutil.copy(source_file, directory)
    else:
        print(f"文件 {source_file} 不存在")
        return
    source_file = os.path.join("../modules/deployment/execution_scripts", "run_cap.py")
    if os.path.exists(source_file):
        shutil.copy(source_file, directory)
    else:
        print(f"文件 {source_file} 不存在")
        return

    # 遍历文件夹下所有python文件，开头加上from apis import *
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".py") and file != "apis_meta.py":
                file_path = os.path.join(root, file)
                with open(file_path, "r+", encoding="utf-8") as f:
                    content = f.read()
                    if not content.startswith("from apis_meta import *"):
                        f.seek(0, 0)
                        f.write("from apis_meta import *\n" + content)
    import re

    sleep_patterns = [
        re.compile(r"(\s*)time\.sleep\(\s*.*?\s*\)"),
        re.compile(r"(\s*)rate\.sleep\(\s*.*?\s*\)"),
    ]

    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".py"):
                file_path = os.path.join(root, file)
                with open(file_path, "r", encoding="utf-8") as f:
                    lines = f.readlines()

                with open(file_path, "w", encoding="utf-8") as f:
                    for line in lines:
                        for pattern in sleep_patterns:
                            match = pattern.match(line)
                            if match:
                                line = f"{match.group(1)}# {line}"
                                break
                        f.write(line)
