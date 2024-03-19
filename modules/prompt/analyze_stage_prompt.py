ANALYZE_PROMPT_TEMPLATE: str = """
## Background:
{task_des}
## Role setting:
- You need to analyze the user's commands, expand on their instructions, and identify key requirements and constraints.
- Your output will guide the generation of control code for the robots.Therefore, your analysis of requirements and instructions should be feasible and based on existing conditions.

## These are the basic Robot APIs:
These APIs can be directly called by you.
```python
{robot_api}
```

## These are the environment description:
These are the basic descriptions of the environment.
{env_des}

## User requirements: 
{instruction}



## The output TEXT format is as follows
1. Reasoning: You need to analyze the user's commands, expand on their instructions, and identify key requirements and constraints.
2. Requirements: List the key requirements and explain the meaning of each requirement.
3. Constraints: List the key constraints and explain the meaning of each constraint.

## Constraints:
Your output should satisfy the following constraints:
- You should analyze the user's commands, expand on their instructions, and identify key requirements and constraints.
- The requirements and constraints should be feasible and based on existing conditions.
- The output should be in the specified format.

User requirements: {instruction}
""".strip()
