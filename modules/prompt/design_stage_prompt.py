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
- Ensure the function name is {function_name}, and set the number of input and output variables as needed.
- All parameters required for the algorithm should be set as input variables with default values.
- The function body content does not need to be provided; simply giving a `pass` is sufficient.
- Take a holistic approach and reuse existing functions as much as possible.
- Task allocation should occur only once at the beginning and must take environmental changes into account, avoiding reliance on any single changing object. The allocation method should be optimal, ensuring no conflicts occur between robots.
- The task allocation can include various types such as positions, lists of positions, or specific angles, based on the requirements of the task.
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
- Ensure the function name is {function_name}, and set the number of input and output variables as needed.
- All parameters required for the algorithm should be set as input variables with default values.
- The function body content does not need to be provided; simply giving a `pass` is sufficient.
- Take a holistic approach and reuse existing functions as much as possible.
- The allocation method for robots should be optimal, ensuring no conflicts occur between them.
- Task allocation will only occur once at the beginning of the task, so the tasks assigned to each robot should take environmental changes into account and avoid relying on any single changing object.
- The robot allocation method should be optimal to avoid conflicts; task allocation occurs once at the beginning and should account for environmental changes without relying on a single changing element.
- The task allocation can include various types such as positions, lists of positions, or specific angles, based on the requirements of the task.
""".strip()
