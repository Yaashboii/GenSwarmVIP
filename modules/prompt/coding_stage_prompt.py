WRITE_FUNCTION_PROMPT_TEMPLATE = """
These are the environment description: 
{env_des}

These are the basic Robot APIs:
{robot_api}

These are existing functions:
{other_functions}

The functions you generate need to comply with the following constraints.
constrains: 
1. Above functions are in the same py file as yours and can be invoked directly using their function names.
2. COMPLETE CODE: Your code will be part of the entire project, so please implement complete, reliable, reusable code snippets.
3. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. 
5. Before using a others functions/modules, make sure to import them first.
6. YOU MUST FOLLOW "Data structures and interface definitions". DONT CHANGE ANY DESIGN.
7. You only need to make sure single function has correct input and output. don't need to consider complex conditions.
8. You can only complete this one function; you cannot generate other Helper functions. If necessary, you can define functions within this function.

Please finish the following function, maximizing the reuse of existing functions:
{function}
""".strip()

WRITE_RUN_PROMPT_TEMPLATE = """
You are a robot, you need to translate the sequence diagram into Python code.
These are the environment description: 
{env_des}

The list of functions you can call is as follows:
from functions imports *
{robot_api}
{function_list}

constrains: 
1. the functions provided are available, you can use them by "from functions import *" 
2. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. 
3. Ensure the generated code contains a 'if __name__ == '__main__':' statement and can be executed directly

The generated result should be in the following fields:
explanation: think step by step. How to translate the sequence diagram into Python code.
python code: Translate the sequence diagram into corresponding Python code. ```python\n <your response>```

The output  TEXT format is as follows:
explanation: <explanation>
python code: <python code>

This is the Sequence Diagram: 
{sequence_diagram}
""".strip()
