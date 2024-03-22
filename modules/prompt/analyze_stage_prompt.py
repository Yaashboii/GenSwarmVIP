ANALYZE_FUNCTION_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You are a function designer. You need to design functions based on user commands and constraint information.
- Your output will guide the generation of control code for the robots. Therefore, the functions you generate should be accurate and feasible, and based on existing conditions.
## These are the basic Robot APIs:
These APIs can be directly called by you.
```python
{robot_api}
```
## These are the environment description:
These are the basic descriptions of the environment.
{env_des}
## Constraints information:
The following are the constraints that the generated functions need to satisfy.
{constraints}
## User commands:
{instruction}
## The output TEXT format is as follows:
```json
{output_template}
```
## Notes:
Your output should satisfy the following notes:
- Analyze what essential functions are needed to implement the user commands.
- Each function should be decoupled from others and closely cooperate, collaborate, and even call each other.
- Each function should be as detailed as possible while also being clear, feasible, and based on existing conditions.
- Each function only needs to implement a small functionality under the overall objective, and one function should not solve multiple problems.
- The output should strictly adhere to the specified format.
- You need to consider which constraints each function should satisfy or, in other words, implement.
- One function can satisfy multiple constraints, and several functions can also implement a single constraint.
- You only need to provide the names of the functions and their constraint information; designing the function bodies is not required.
- The constraints section of each function needs to select the corresponding Constraints Name from the Constraints information.
- If a function calls other functions, it is considered that this function has satisfied the constraints of the called functions, and there is no need for this function to include the constraints information of the called functions.
- Each constraint in the Constraints information should be satisfied by one of the functions generated in your function list, without any omissions.
""".strip()

ANALYZE_CONSTRAINT_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- Analyze what constraints should be met by the code designed to execute the user's commands.
- Your output will be used as a standard to check the final generated code, so you need to ensure that your constraints are checkable, feasible, and specifically targeted towards the generated code.
These APIs can be directly called by you.
```python
{robot_api}
```
## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## The output TEXT format is as follows:
```json
{output_template}
```
## Constraints:
Your output should satisfy the following constraints:
- Analyze what constraints should be met by the code designed to execute the user's commands.
- Each constraint should be feasible, and specifically targeted towards the generated code.
- The output should strictly adhere to the specified format..
""".strip()

CONSTRAIN_TEMPLATE: str = """
{
  "paraphrase": "Paraphrase the user's commands in your own words.",
  "constraints": [
    {
      "name": "Constraint name",
      "description": "Extremely detailed description of the constraint."
    },
    ... 
  ]
}
""".strip()

FUNCTION_TEMPLATE: str = """
{
  "paraphrase": "Paraphrase the user's commands here",
  "functions": [
    {
      "name": "Function name",
      "description": "Extremely detailed description of the function.",
      "constraints": [
        "Name of the constraint that this function needs to satisfy"
        // More constraints can be added as needed
      ]
    }
    // More functions can be added as needed
  ]
}
""".strip()

PARAMETER_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You need to further clarify which parameters the entire code will use based on the demand analysis and constraint analysis.
- These parameters should be global, and the code written should use these parameters as currently defined.

## These are the environment description:
{env_des}

## Requirements and Constraints:
{requirements_constraints}

## The output TEXT format is as follows
1. Reasoning: Infer all parameters that might be used throughout the entire task process, with as much detail as possible.
2. Parameters: ```python\n parameter_name: parameter_type = default_value\n...```

## Constraints:
Your output should satisfy the following constraints:
- You should further clarify which parameters the entire code will use based on the demand analysis and constraint analysis.
- Strictly follow the specified format.
"""
