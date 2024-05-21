DEBUG_PROMPT = """
## Background:
{task_des}

## Role setting:
-It's now the phase to run the code, your task is to find the erroneous part based on the compiler's traceback feedback, and modify it.

## These are the environment description:
{env_des}

## These are the basic Robot APIs:
```python
{robot_api}
```
## These are the functions that mentioned in the error message:
{mentioned_functions}

## These are the error messages:
{error_message}

## Task
According to the error message, make modifications based on the existing foundation, and output the modified function in its entirety.
The output TEXT format is as follows:
Reasoning: what caused the error, and how did you fix it? Provide serval fixes and choose the best one.
code:
```python
import ...(if necessary)


function_name(...):
    ...


...
```

## Notes:
1. Only allowed to modify errors, not allowed to modify function names as well as the input and output of the function.
2. Output the complete code of the entire function, not just a part of it that's been omitted.
3. Rewrite all functions that need modifications.
4. Keep the original code in the function as unchanged as possible, only modifying the parts that are incorrect.
5. The output should be in the specified format.
""".strip()

HUMAN_FEEDBACK_PROMPT_TEMPLATE = """
## Background:
{task_des}

## Role setting:
- After running the code, there are some issues. You need to modify based on the feedback from users.

## These are the environment description:
{env_des}

## These are the basic Robot APIs:
```python
{robot_api}
```

## These are the functions that can be modified:
```python
{functions}
```

## These are the user's feedback:
{feedback}

## Output format:
### Reasoning: you should analyze in the step:
    - What is the problem?
    - Which part of the code is problematic?
    - What is the solution?
### Code:
```python
import ...(if necessary)

function_name(...):
    ...

## Notes:
- Do not modify the function name.
- The input and output of the function could be modified.But you should make sure other functions that call this function can still work.
- Output the complete code of the entire function, not just a part of it that's been omitted.
- Rewrite all functions that need modifications.
- Keep the original code in the function as unchanged as possible, only modifying the parts that are incorrect.
- The output should be in the specified format.
""".strip()

GRAMMAR_CHECK_PROMPT_TEMPLATE = """
## Background:
{task_des}

## Role setting:
- The code you wrote encountered issues during syntax checking. Now, you need to modify the code according to the problems identified to meet the requirements.

## These are the environment description:
{env_des}

## These are the basic Robot APIs:
```python
{robot_api}
```

## These are the functions:
```python
{functions}
```

## These are the error messages:
{error}

## Output format:
### Reasoning: you should analyze in the step:
    - What is the problem?
    - How was the problem generated?
    - What is the solution?
    
### Code:
```python
import ...(if necessary)

function_name(...):
    ...

## Notes:
- Do not modify the function name.
- The input and output of the function could be modified.But you should make sure other functions that call this function can still work.
- Output the complete code of the entire function, not just a part of it that's been omitted.
- Rewrite all functions that need modifications.
- Keep the original code in the function as unchanged as possible, only modifying the parts that are incorrect.
- The output should be in the specified format.
""".strip()
