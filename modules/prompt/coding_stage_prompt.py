WRITE_FUNCTION_PROMPT_TEMPLATE = """
You need to complete this function, making maximum reuse of existing functions.
These are the basic Robot APIs:
{robot_api}
These are existing functions:
{other_functions}

The functions you generate need to comply with the following constraints.
constrains: 
1. All environment APIs are readily accessible and can be invoked directly using their function names.
2. COMPLETE CODE: Your code will be part of the entire project, so please implement complete, reliable, reusable code snippets.
3. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. 
5. Before using a external variable/module, make sure you import it first. But Environment APIs are readily accessible and can be invoked directly using their function names.
6. YOU MUST FOLLOW "Data structures and interface definitions". DONT CHANGE ANY DESIGN.
7. You only need to make sure single function has correct input and output. don't need to consider complex conditions.
8. You can only complete this one function; you cannot generate other Helper functions. If necessary, you can define functions within this function.

Please finish the following function:
{function}
""".strip()

WRITE_RUN_PROMPT_TEMPLATE = """
You are a robot, you need to translate the sequence diagram into Python code.
These are the environment description: {env_des}

constrains: 
1. You can't define any new functions.
3. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. 
3. All functions that appear in the sequence diagram do not need to be imported. However, if you need to use external libraries, you must import them correctly.
4. Ensure the generated code contains a 'if __name__ == '__main__':' statement and can be executed directly

The generated result should be in the following fields:
explanation: think step by step. How to translate the sequence diagram into Python code.
python code: Translate the sequence diagram into corresponding Python code. ```python\n <your response>```

The output format is as follows:
{{
explanation: <explanation>
python code: <python code>
}}

Sequence Diagram: {sequence_diagram}
""".strip()
