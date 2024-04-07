import os
import re
import ast
import shutil
import time

import cv2
import rospy
from typing import Any
from enum import Enum
from std_srvs.srv import SetBool


class TestResult(Enum):
    HALF_PASS = 1
    ALL_PASS = 2
    NOT_PASS = 3


class BugSource(Enum):
    CODE = 1
    TEST_CODE = 2


class DesignPattern(Enum):
    FUNCTION = 1
    SEQ_DIAGRAM = 2


class CodeMode(Enum):
    WRITE_FUNCTION = 1
    WRITE_RUN = 2


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


def check_file_exists(directory, filename):
    file_path = os.path.join(directory, filename)
    return os.path.isfile(file_path)


def write_file(directory, filename, content, mode='w'):
    from modules.framework.workflow_context import logger

    try:
        file_path = os.path.join(directory, filename)

        with open(file_path, mode) as file:
            file.write(content)
        operation = "written" if mode == 'w' else "appended"
        if operation == 'written':
            logger.log(f"File {operation}: {file_path}", level='info')

    except FileNotFoundError:
        logger.log(f"File not found: {file_path}", level='error')
    except Exception as e:
        logger.log(f"Error writing file: {e}", level='error')


def copy_folder(source_folder, destination_folder):
    try:
        # Copy the entire folder and its contents
        shutil.copytree(source_folder, destination_folder)
    except Exception as e:
        raise Exception(f"Error copying folder: {e}")


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
        error_message = f"Error: No '{lang}' code block found in the text."
        raise ValueError(error_message)
    return code


def extract_function_definitions(source_code):
    parsed_ast = ast.parse(source_code)

    def reconstruct_function_definition(function_node):
        defaults_start_index = len(function_node.args.args) - len(function_node.args.defaults)

        parameters = [
            ast.unparse(arg) + (
                f'={ast.unparse(function_node.args.defaults[i - defaults_start_index])}' if i >= defaults_start_index else '')
            for i, arg in enumerate(function_node.args.args)
        ]

        func_header = f"def {function_node.name}({', '.join(parameters)}):"
        docstring = ast.get_docstring(function_node)
        docstring_part = ''
        if docstring:
            indented_docstring = '\n'.join('    ' + line for line in docstring.split('\n'))
            docstring_part = f'    """\n{indented_docstring}\n    """\n'
        body_part = ''
        return f"{func_header}\n{docstring_part}{body_part}"

    function_definitions = [reconstruct_function_definition(node) for node in ast.walk(parsed_ast) if
                            isinstance(node, ast.FunctionDef)]

    return function_definitions


def extract_imports_and_functions(source_code):
    parsed_ast = ast.parse(source_code)

    imports = []
    functions = []

    for node in parsed_ast.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    import_str = f"import {alias.name}"
                    if alias.asname:
                        import_str += f" as {alias.asname}"
                    imports.append(import_str)
            elif isinstance(node, ast.ImportFrom):
                module = node.module if node.module else ''
                import_from_str = "from {} import ".format(module)
                names_with_as = []
                for alias in node.names:
                    if alias.asname:
                        names_with_as.append(f"{alias.name} as {alias.asname}")
                    else:
                        names_with_as.append(alias.name)
                import_from_str += ", ".join(names_with_as)
                imports.append(import_from_str)
        elif isinstance(node, ast.FunctionDef):
            func_def = ast.unparse(node).strip()
            if func_def:
                functions.append(func_def)

    return imports, functions


def extract_top_level_function_names(code_str: str) -> list[str]:
    tree = ast.parse(code_str)

    def add_parent_references(node, parent=None):
        node.parent = parent
        for child in ast.iter_child_nodes(node):
            add_parent_references(child, node)

    add_parent_references(tree)

    function_names = []

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and isinstance(node.parent, ast.Module):
            function_names.append(node.name)

    return function_names


def combine_unique_imports(import_list):
    unique_imports = set()

    for import_str in import_list:
        import_lines = import_str.splitlines()
        for import_line in import_lines:
            unique_imports.add(import_line.strip())

    combined_imports = "\n".join(sorted(unique_imports))

    return combined_imports


def find_function_name_from_error(file_path, error_line):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        error_code_line = lines[error_line - 1].strip()
        for i in range(error_line - 2, -1, -1):
            if lines[i].strip().startswith('def '):
                function_name = lines[i].strip().split('(')[0].replace('def ', '')
                return function_name, error_code_line
    return None, error_code_line


def check_grammar(file_path: str):
    import subprocess
    command = [
        'pylint',
        # '--disable=W,C,I,R --enable=E,W0612',
        '--disable=W,C,I,R ',
        file_path
    ]

    try:
        process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        result = process.stdout + process.stderr

        pattern = re.compile(r"(.*?):(\d+):(\d+): (\w+): (.*) \((.*)\)")
        matches = pattern.findall(result)

        errors = []
        for match in matches:
            file_path, line, column, error_code, error_message, _ = match
            errors.append({
                "file_path": file_path,
                "line": int(line),
                "column": int(column),
                "error_code": error_code,
                "error_message": error_message
            })

        return errors
    except Exception as e:
        from modules.framework.workflow_context import logger
        logger.log(f"Error occurred when check grammar: {e}", level='error')
        raise Exception(f"Error occurred when check grammar:{e}")


def call_reset_environment(data: bool):
    """

    Args:
        data (bool): Whether to render the environment
    """
    from modules.framework.workflow_context import logger

    if not rospy.core.is_initialized():
        rospy.init_node('reset_environment_client', anonymous=True)

    rospy.wait_for_service('/reset_environment')
    try:
        reset_environment = rospy.ServiceProxy('/reset_environment', SetBool)
        resp = reset_environment(data)
        return resp.success, resp.message
    except rospy.ServiceException as e:
        logger.log(f"Service call failed: {e}", level='error')


def get_param(param_name):
    try:
        return rospy.get_param(param_name)
    except KeyError:
        print(f"Parameter not found: {param_name},retrying...")
        time.sleep(1)
        return get_param(param_name)


def set_param(param_name, param_value):
    from modules.framework.workflow_context import logger
    rospy.set_param(param_name, param_value)
    logger.log(f"Parameter set: {param_name} = {param_value}", level='info')


def generate_video_from_frames(frames_folder, video_path, fps=15):
    from modules.framework.workflow_context import logger
    logger.log(f"Generating video from frames in {frames_folder}...")
    try:
        frame_files = sorted(
            [file for file in os.listdir(frames_folder) if re.search(r'\d+', file)],
            key=lambda x: int(re.search(r'\d+', x).group())
        )
    except Exception as e:
        logger.log(f"Error reading frames: {e}", level='error')
        return

    if not frame_files:
        logger.log("No frames found", level='error')
        return
    frame_files = [os.path.join(frames_folder, file) for file in frame_files]

    frame = cv2.imread(frame_files[0])
    height, width, layers = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    video = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    for frame_file in frame_files:
        video.write(cv2.imread(frame_file))

    cv2.destroyAllWindows()
    video.release()
    logger.log(f"Video generated: {video_path}", level='info')
