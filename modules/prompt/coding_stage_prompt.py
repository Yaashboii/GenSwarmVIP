WRITE_FUNCTION_PROMPT_TEMPLATE = """
## Background:
{task_des}

## Role setting:
- Your task is to complete this predefined function according to the description.

## These are the environment description: 
{env_des}

## These are the basic Robot APIs:
```python
{robot_api}
```

## These are the functions you can call directly even if they are not implemented now:
```python
{other_functions}
```

## These are the constraints that need to be satisfied in the implementation of the function:
{constraints}

## The output TEXT format is as follows:
```python
import ...(if necessary)
{function_content}
    ...(function body,you need to complete it)
```

## Notes: 
1. The Robot API and existing functions can be called directly without the need  imports.
2. Write very detailed descriptions of the function.
3. Reduce the occurrence of errors by writing standard-compliant, correct functions.
4. Set default value in input parameters: Any adjustable parameters should be taken as input parameters of the function. Always set a default value for each parameter.
5. You can only complete this one function; you cannot generate other Helper functions. If necessary, you can define functions within this function.
6. Consider reuse or collaboration with existing functions; this one function is just a link in the entire control system.
7. Avoid using global variables, and avoid using the same variable name as the global variable in the function.
8. Import the required modules at the beginning of the file, and do not import them in the function. 
9. Make sure the functions you generate meet the constraints.
10. Don't complete other functions, just complete the function in the specified format.
""".strip()

WRITE_RUN_PROMPT_TEMPLATE = """
## Task description:
You are a robot, you need to translate the sequence diagram into Python code.

## These are the environment description: 
{env_des}

## The list of functions you can call is as follows:
```python
from functions imports *
{robot_api}
{function_list}
```

## This is the Sequence Diagram: 
{sequence_diagram}

## Constrains: 
1. the functions provided are available, you can use them by "from functions import *" 
2. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. 
3. Ensure the generated code contains a 'if __name__ == '__main__':' statement and can be executed directly
4. Write complete code, your code will not be modified, and if you can't write complete code, simplify the function.

## The generated result should be in the following fields:
1. explanation: think step by step. How to translate the sequence diagram into Python code.
2. python code: Translate the sequence diagram into corresponding Python code. ```python\n <your response>```

## The output  TEXT format is as follows:
1. explanation: <explanation>
2. python code: <python code>
""".strip()
