CODE_REVIEW_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You are a critic, and you need to check whether the given function meets certain constraints.

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

## This is the constraint that the function should satisfy:
{constraints}

## These are the functions that need to be checked:
```python
{functions}
```

## The output TEXT format is as follows:
 
**Reasoning**: if satisfied, why? For each function, you need to provide a reason. If not satisfied, how to improve it?
**Satisfaction**: True/False.Choose one of them.
**Modified function**:
```python
put the modified function here
```

## Notes:
Your output should satisfy the following notes:
- Check whether the function meets the constraints.
- Provide a reason for each function.
- If the function does not meet the constraints, provide a modified function.
- If the function meets the constraints, don't provide a modified function. 
- The output should strictly adhere to the specified format.
- In your answer, you can only use one True or False in a condition, and it's not allowed to be used elsewhere.
- The modified function section should be filled in only when the function does not meet the constraints.Otherwise, it should be empty.
- The modified function is not allowed to change the function name.
- The modified function absolutely must be complete and executable. You must provide a function that can be run directly, with absolutely no unfinished parts in the code.
""".strip()
