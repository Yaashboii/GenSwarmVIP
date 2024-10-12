LOCAL_CODE_REVIEW_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You are a critic, You should now check if there are any bugs in the functions written by other agents or if there have been any incorrect calls.

## These are the basic Robot APIs:
These APIs can be directly called by you.
```python
{robot_api}
```

## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## These are the existing functions that you can directly call:
```python
{other_functions}
```

## These are the functions that need to be checked:
```python
{function_content}
```

## These are the constraints that the function should satisfy:
{constraints}


## The output TEXT format is as follows:
Reasoning: think step by step, whether the to be checked function has any bugs or incorrect calls.In the reasoning section, you can directly write the code without using ```python```.

Modified function:
```python
import ... (import the necessary modules,if any)
def {function_name}(...):
    # put the modified function body here,please output the complete and correct function, and only modify the parts that are incorrect.
```

## Notes:
Your output should satisfy the following notes:
- Carefully check if there are any bugs in this function, or if there are any logical inconsistencies.
- Check if the constraints are met.
- Check if the call to each sub-function is correct.
- Check that all defined variables are used.
- Strictly adhere to the specified format.
- During the Reasoning section, '''python''' is not allowed. You can only use '''python''' in the Modified function section. You can directly write the code in the Reasoning section.
- The modified function section should be filled in only when the function has bugs or incorrect calls. Otherwise, this section should not appear.
- If the function needs to be rewritten, please ensure that the rewritten function name is {function_name}.
- If you need to rewrite a function, please output the complete and correct function, and only modify the parts that are incorrect.
- You should carefully consider and ensure that your revised version is correct.
- If there is an error in this function, please provide the erroneous line of code in the "reasoning" section, along with suggestions for how it could be corrected.
- If the function outputs velocity, then this velocity must be normalized.
- The current task does not necessarily require a global allocator. If needed, please use the corresponding API to obtain the assigned task. If there is no corresponding API, then the current task does not require a global allocator.
- When using the API to obtain the task assigned to the current robot, it only needs to be called once. Repeated calls will not be useful, as the allocation is done only once at the beginning based on the robot's initial state.
- Preserve the function's docstring, with the option to modify its content.
- The current task does not necessarily require a global allocator. If needed, please use the corresponding API to obtain the assigned task. If there is no corresponding API, then the current task does not require a global allocator.
- If the revised function name is run_loop,here is some special notes:
  - The run_loop function is the main function that the ser calls to run the entire task. It is the entry point for the entire task.
  - The run_loop function should contain all the necessary logic to complete the task.
  - Calling time.sleep or any other method to limit frequency is not allowed, as the underlying API has determined a set frequency.
  - The while loop should be used to ensure that the function is called continuously and that the robot can update its observation data in real time.If the function is not implemented in other functions,the run_loop function should be used to achieve this.
  - The while loop must be endless, and the function must be able to run continuously.
""".strip()

GLOBAL_CODE_REVIEW_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You are a critic, You should now check if there are any bugs in the functions written by other agents or if there have been any incorrect calls.

## These are the basic Robot APIs:
These APIs can be directly called by you.
```python
{robot_api}
```

## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## These are the existing functions that you can directly call:
```python
{other_functions}
```

## These are the functions that need to be checked:
```python
{function_content}
```

## These are the constraints that the function should satisfy:
{constraints}


## The output TEXT format is as follows:

Reasoning: think step by step, whether the to be checked function has any bugs or incorrect calls.In the reasoning section, you can directly write the code without using ```python```.

Modified function:
```python
import ... (import the necessary modules,if any)
def {function_name}(...):
    # put the modified function body here,please output the complete and correct function, and only modify the parts that are incorrect.
```

## Notes:
Your output should satisfy the following notes:
- Carefully check if there are any bugs in this function, or if there are any logical inconsistencies.
- Check if the constraints are met.
- Check if the call to each sub-function is correct.
- Check that all defined variables are used.
- Strictly adhere to the specified format.
- During the Reasoning section, '''python''' is not allowed. You can only use '''python''' in the Modified function section. You can directly write the code in the Reasoning section.
- The modified function section should be filled in only when the function has bugs or incorrect calls. Otherwise, this section should not appear.
- If the function needs to be rewritten, please ensure that the rewritten function name is {function_name}.
- If you need to rewrite a function, please output the complete and correct function, and only modify the parts that are incorrect.
- You should carefully consider and ensure that your revised version is correct.
- If there is an error in this function, please provide the erroneous line of code in the "reasoning" section, along with suggestions for how it could be corrected.
- If the function outputs velocity, then this velocity must be normalized.
- The current task does not necessarily require a global allocator. If needed, please use the corresponding API to obtain the assigned task. If there is no corresponding API, then the current task does not require a global allocator.
- When using the API to obtain the task assigned to the current robot, it only needs to be called once. Repeated calls will not be useful, as the allocation is done only once at the beginning based on the robot's initial state.
- Preserve the function's docstring, with the option to modify its content.
- The allocation method for robots should be optimal, ensuring no conflicts occur between them.
- If the revised function name is allocate_run ,here is some special notes:
    - The allocate_run function is the main function that the ser calls to run the entire task. It is the entry point for the entire task.
    - The allocate_run function should contain all the necessary logic to complete the task.
    - Calling time.sleep or any other method to limit frequency is not allowed, as the underlying API has determined a set frequency.
    - Task allocation will only occur once at the beginning of the task, so the tasks assigned to each robot should take environmental changes into account and avoid relying on any single changing object.

""".strip()
