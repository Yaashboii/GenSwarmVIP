ANALYZE_SKILL_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You are a function designer. You need to design functions based on user commands and constraint information.

## These are the environment description:
These are the basic descriptions of the environment.
{env_des}


## These APIs can be directly called by you:
There are two types of APIs: local and global.
where local APIs can only be called by the robot itself, and global APIs can be called by an centralized controller.

### local APIs:
```python
{local_api}
```

### global APIs:
```python
{global_api}
```

## Constraints information:
The following are the constraints that the generated functions need to satisfy.
{constraints}
## User commands:
{instruction}
## The output TEXT format is as follows:
{output_template}

## Notes:
- Analyze the essential functions needed to implement the user commands.
- Each function should be decoupled from others but able to cooperate, collaborate, and call each other when necessary.
- Each function should be as detailed as possible while remaining clear, feasible, and based on existing conditions.
- Each function should implement a small part of the overall objective; no single function should solve multiple problems.
- Each function must satisfy the relevant constraints, meaning it implements that constraint.
- One function can satisfy multiple constraints, and multiple functions can be designed to implement a single constraint.
- Only the names of the functions, their constraints, and their call relationships are required; specific implementation details are not needed.
- The inter-call relationships among these functions must be determined.
- There should be no functional redundancy among these functions, with each function having a distinct responsibility.
- Analyze only the constraints that the current function itself must meet; constraints related to functions it calls are beyond the scope of the current function.
- Each constraint must be fulfilled by one of the functions listed, without any omissions.
- If a function outputs velocity, the velocity must be normalized.
- Distinguish which skills should run on a centralized allocator and which should run on individual robots.
- The skill design in tasks should be divided into two categories, and the appropriate skill type should be selected based on the specific requirements of the task.
- The allocation method for robots should be optimal, ensuring no conflicts occur between them.
- Task allocation will only occur once at the beginning of the task, so the tasks assigned to each robot should take environmental changes into account and avoid relying on any single changing object.

""".strip()

ANALYZE_CONSTRAINT_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You need to analyze what functional constraints are needed to meet the user's requirements.

## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## These APIs can be directly called by you.
There are two types of APIs: local and global.
where local APIs can only be called by the robot itself, and global APIs can be called by an centralized controller.

### local APIs:
```python
{local_api}
```

### global APIs:
```python
{global_api}
```

## User commands:
{instruction}


## The output TEXT format is as follows:
{output_template}

## Notes:
Your output should satisfy the following notes:
- Constraints should not be too simple or too complex; the amount of code required to implement each constraint should be similar.
- Constraints should be practical and achievable through writing code.
- The constraints are targeted at individual robots themselves, not all robots as a whole. However, if each robot meets the constraints, collective behavior can be achieved.
- Each constraint will correspond to at least one executable function, and the combination of all constraints can meet the user's needs.
- Proper analysis of the task should guide how to design constraints, which constraints to design, to fulfill the user's task requirements.
- You need to understand the existing APIs. The capabilities provided by these APIs have already been implemented, which means the robot can directly call these APIs without considering the underlying implementation or the constraints involved.
- Analyze the core tasks proposed by the user and perform a functional decomposition of these core tasks.
- There's no need to regenerate existing constraints; you only need to consider what new constraints are required.
- These constraints should be significant and mutually independent.
- If the user's instruction involves specific numerical values, you should retain these values in the description of the constraints.
- The output should strictly adhere to the specified format.
""".strip()

CONSTRAIN_TEMPLATE: str = """
##reasoning: (you should think step by step, and analyze the constraints that need to be satisfied in the task.place the analysis results at here.)
```json
{
  "constraints": [
    {
      "name": "Constraint name",
      "description": "Description of the constraint.(If the user's requirements involve specific numerical values, they should be reflected in the description. )"
    },
  ]
}
```
""".strip()

FUNCTION_TEMPLATE: str = """
##Reasoning: (Think step by step, and analyze the functions that need to be implemented in the task. First, analyze the global functions, then analyze the local functions.)
```json
{
  "functions": [
    {
      "name": "Function name",//Function names use snake case.
      "description": "Description of the function,contains the function's input and output parameters",
      "constraints": [
        "Name of the constraint that this function needs to satisfy"
        // More constraints can be added as needed
      ]
      "calls": [
        "Function name that this function calls(Robot API is also included)"
      ]
      "scope": "local/global"
    }
    // More functions can be added as needed
  ]
}
```
""".strip()
