DESIGN_LOCAL_FUNCTION_PROMPT_TEMPLATE = """
## Background:
{task_des}
## Role setting:
- Your task is to refine the designed function {function_name} based on the existing descriptions while keeping the function name unchanged.

## These are the environment description:
{env_des}

## Existing robot APIs:
```python
{robot_api}
```

## These are the existing functions' descriptions and names:
{other_functions}

## These are the constraints that this function should satisfy.
{constraints}

## The output TEXT format is as follows:
### Reasoning: (reason step by step about how to design this function)
### Code:
```python
def {function_name}(input1, input2, ...):
    '''
    Description:Refine this description '{function_des}' in detail to guide the generation of the function and put it at here.

    params:
        input1: type, description
        input2: type, description
        ...
    return:
        type, description
    '''
    pass
```

## Notes:
- You need to enhance the existing function descriptions by adding more details.
- Keep the function name {function_name} unchanged; the number of input and output variables is set as needed.
- All parameters required for the algorithm should be set as input variables with default values.
- The function does not need to provide the content of the function body; just giving a `pass` is suffice.
- Take a holistic approach and reuse existing functions as much as possible.
- Make sure the function name is {function_name}.
- The current task does not necessarily require a global allocator. If needed, please use the corresponding API to obtain the assigned task. If there is no corresponding API, then the current task does not require a global allocator.
- The output should be in the specified format.
""".strip()

DESIGN_GLOBAL_FUNCTION_PROMPT_TEMPLATE = """
## Background:
{task_des}

## Role setting:
- Your task is to refine the designed function {function_name} based on the existing descriptions while keeping the function name unchanged.

## These are the environment description:
{env_des}

## Existing APIs:
```python
{robot_api}
```

## These are the existing functions' descriptions and names:
{other_functions}

## These are the constraints that this function should satisfy.
{constraints}


## The output TEXT format is as follows:
### Reasoning: (reason step by step about how to design this function)
### Code:
```python
def {function_name}(input1, input2, ...):
    '''
    Description:Refine this description '{function_des}' in detail to guide the generation of the function and put it at here.

    params:
        input1: type, description
        input2: type, description
        ...
    return:
        type, description
    '''
    pass
```

## Notes:
- You need to enhance the existing function descriptions by adding more details.
- Keep the function name {function_name} unchanged; the number of input and output variables is set as needed.
- All parameters required for the algorithm should be set as input variables with default values.
- The function does not need to provide the content of the function body; just giving a `pass` is suffice.
- Take a holistic approach and reuse existing functions as much as possible.
- Make sure the function name is {function_name}.
- The allocation method for robots should be optimal, ensuring no conflicts occur between them.
- Task allocation will only occur once at the beginning of the task, so the tasks assigned to each robot should take environmental changes into account and avoid relying on any single changing object.
- The output should be in the specified format.
""".strip()
