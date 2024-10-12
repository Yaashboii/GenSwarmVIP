WRITE_GLOBAL_FUNCTION_PROMPT_TEMPLATE = """
## Background:
{task_des}
## Role setting:
- Your task is to complete this predefined function according to the description.

## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## These are GLOBAL APIs:
```python
{robot_api}
```

## These are the functions you can call directly even if they are not implemented now:
```python
{other_functions}
```

## These are the constraints that need to be satisfied in the implementation of the function:
{constraints}

## Task
Complete the following function. The output TEXT format is as follows:
### Reasoning: (reason step by step about how to implement this function.)
### Code:
```python
import ...(if necessary)
{function_content}
    ...(function body,you need to complete it)
```
## Notes:
- The global API and existing functions can be called directly without the need imports.
- You need to generate bug-free, directly invocable function code according to Google's coding standards.
- In the code you generated, the use of “raise error” and "assert" is strictly prohibited. When you encounter an issue, you must resolve it within the function itself instead of raising an error or using assert.
- Set default value in input parameters: Any adjustable parameters should be taken as input parameters of the function. Always set a default value for each parameter.
- You can only complete the functions specified in Task according to the specified format; you cannot generate other Helper functions. If necessary, you can define functions within this function.
- Take a holistic approach and reuse **existing functions as much as possible**; this one function is just a link in the entire control system.
- Avoid using global variables, and avoid using the same variable name as the global variable in the function.
- Preserve the function's docstring, with the option to modify its content.
- Avoid using While loop in the function body.
- Avoid raising exceptions in the function body.
- If there are issues with the function definition or docstring in the task,and you need to modify them, please make sure the function name remains the same.
- Import the required modules before the function name, and do not import them in the function body.
- The allocation method for robots should be optimal, ensuring no conflicts occur between them.
- Task allocation will only occur once at the beginning of the task, so the tasks assigned to each robot should take environmental changes into account and avoid relying on any single changing object.
- The output should be in the specified format.
- Make sure the functions you generate meet the constraints.
""".strip()

WRITE_LOCAL_FUNCTION_PROMPT_TEMPLATE = """
## Background:
{task_des}
## Role setting:
- Your task is to complete this predefined function according to the description.

## These are the environment description:
These are the basic descriptions of the environment.
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

## Task
Complete the following function. The output TEXT format is as follows:
### Reasoning: (reason step by step about how to implement this function.)
### Code:
```python
import ...(if necessary)
{function_content}
    ...(function body,you need to complete it)
```
## Notes:
- The Robot API and existing functions can be called directly without the need imports.
- You need to generate bug-free, directly invocable function code according to Google's coding standards.
- In the code you generated, the use of “raise error” and "assert" is strictly prohibited. When you encounter an issue, you must resolve it within the function itself instead of raising an error or using assert.
- Set default value in input parameters: Any adjustable parameters should be taken as input parameters of the function. Always set a default value for each parameter.
- You can only complete the functions specified in Task according to the specified format; you cannot generate other Helper functions. If necessary, you can define functions within this function.
- Take a holistic approach and reuse **existing functions as much as possible**; this one function is just a link in the entire control system.
- Avoid using global variables, and avoid using the same variable name as the global variable in the function.
- Preserve the function's docstring, with the option to modify its content.
- Avoid using While loop in the function body.
- Avoid raising exceptions in the function body.
- If there are issues with the function definition or docstring in the task,and you need to modify them, please make sure the function name remains the same.
- Import the required modules before the function name, and do not import them in the function body.
- If the function outputs velocity, then this velocity must be normalized.
- The current task does not necessarily require a global allocator. If needed, please use the corresponding API to obtain the assigned task. If there is no corresponding API, then the current task does not require a global allocator.
- Make sure the functions you generate meet the constraints.
""".strip()

WRITE_GLOBAL_RUN_PROMPT_TEMPLATE = """
## Background:
{task_des}

## Role setting:
You need to write an interface function that calls the existing global function to assign appropriate strategies to each robot.
## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## These are the basic global APIs:
```python
{robot_api}
```

## These are the functions generated by other assistants,you can call them directly:
```python
{functions}
```

## Task
Complete the following function. The output TEXT format is as follows:
### Reasoning: (reason step by step about how to implement this function.)
### Code:
```python
import ...(if necessary)
def allocate_run():
    '''
    Description:...

    params:
        None
    return:
        dict: A dictionary containing the robot id and the policy assigned to it.
         - key: int, the robot id.
         - value: any data type, e.g. tuple, list,float etc., decided by the task type.
    '''
    ...(function body,you need to complete it)
```
### Return format:
```json
{template}
```

## Notes:
- The API and existing functions can be called directly without the need imports.
- You need to generate bug-free, directly invocable function code according to Google's coding standards.
- You can only complete the functions specified in Task according to the specified format; you cannot generate other Helper functions. If necessary, you can define functions within this function.
- You can only call these existing functions and global Api, and you cannot define complex logic on your own,just call the existing functions.
- Import the required modules before the function name, and do not import them in the function body.
- Use the existing high-level functions.
- The allocation method for robots should be optimal, ensuring no conflicts occur between them.
- Task allocation will only occur once at the beginning of the task, so the tasks assigned to each robot should take environmental changes into account and avoid relying on any single changing object.
- You should populate the output format based on the code you've written, as this output format will impact the code development for the Robots.
- Ensure that this function can be executed to completion and is able to return the expected output.
- Strictly follow the specified format.
"""

GLOBAL_RUN_OUTPUT_TEMPLATE = """
{
    "key":
        {
            "type":"int",
            "description":"the robot id."
        },
    "value":
        {
            "type":"any data type e.g. tuple[float,float],dict[str,tuple[float,float]]], list[tuple[float,float]],float, etc., decided by the task type.",
            "description":"the task assigned to the robot."
        }
}
"""

WRITE_LOCAL_RUN_PROMPT_TEMPLATE = """
## Background:
{task_des}
## Role setting:
- Your task is to create an interface function for users to call, based on existing functions written by other assistants. Users only need to call this function to complete the predetermined task.

## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## These are the basic Robot APIs:
```python
{robot_api}
```

## These are the functions generated by other assistants,you can call them directly:
```python
{functions}
```

## Task
Complete the following function. The output TEXT format is as follows:
### Reasoning: (reason step by step about how to implement this function.)
### Code:
```python
import ...(if necessary)
def run_loop():
    '''
    Description: An interface function for users to call, based on existing functions written by other assistants. Users only need to call this function to complete the predetermined task.

    params:
        None
    return:
        None
    '''
    ...(function body,you need to complete it)
```

## Notes:
- The Robot API and existing functions can be called directly without the need imports.
- You need to generate bug-free, directly invocable function code according to Google's coding standards.
- You can only complete the functions specified in Task according to the specified format; you cannot generate other Helper functions. If necessary, you can define functions within this function.
- You can only call these existing functions and RobotApi, and you cannot define complex logic on your own.
- Import the required modules before the function name, and do not import them in the function body.
- You need to ensure that the entire system can update observation data in real time and issue control speeds in real time based on the observation data. If it is not implemented in other functions, you need to achieve this through a While loop .
- Use the existing high-level functions.
- Strictly follow the specified format.
- Using time.sleep to limit the frequency of the loop,robot control should be executed at a frequency of 100Hz.
""".strip()
