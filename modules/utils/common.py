import os
import re
from typing import Any
from enum import Enum

from const import ENV_CODE, WORKSPACE_ROOT


class TestResult(Enum):
    PASS = 1
    NOT_PASS= 2

class BugSource(Enum):
    CODE = 1
    TEST_CODE = 2


def get_class_name(cls) -> str:
    """Return class name"""
    return f"{cls.__name__}"


def any_to_str(val: Any) -> str:
    """Return the class name or the class name of the object, or 'val' if it's a string type."""
    if isinstance(val, str):
        return val
    elif not callable(val):
        return get_class_name(type(val))
    else:
        return get_class_name(val)


def write_file(filename, content, directory=WORKSPACE_ROOT):
    if not os.path.exists(directory):
        os.makedirs(directory)
        write_file(filename='env.py', content=ENV_CODE)
    file_path = os.path.join(directory, filename)
    with open(file_path, 'w') as file:
        file.write(content)

    return file_path

def check_file_exists(directory, filename):
    file_path = os.path.join(directory, filename)
    return os.path.exists(file_path)

def read_file(directory, filename):
    file_path = os.path.join(directory, filename)
    try:
        with open(file_path, 'r') as file:
            file_content = file.read()
        return file_content
    except FileNotFoundError:
        return f"File not found: {file_path}"


def parse_code(text: str, lang: str = "python") -> str:
    pattern = rf"```{lang}.*?\s+(.*?)```"
    match = re.search(pattern, text, re.DOTALL)
    if match:
        code = match.group(1)
    else:
        raise Exception
    return code
