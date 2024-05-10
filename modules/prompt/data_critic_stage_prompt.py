OUTPUT_FORMAT = """
{
    "constraint1": {
        "reasoning": "Reasoning",
        "result": "True/False",
    },
    ...
    'constraintN': {
        "reasoning": "Reasoning",
        "result": "True/False",
    }
}
"""

FILTER_CONSTRAINTS_TEMPLATE: str = """
## Task description:
{task_des}
## Role setting:
Now that the code execution is complete, we've obtained all the robot trajectory data, as well as the position and radius data of obstacles in space. Based on this data, you need to determine which constraints can be satisfied.

## Apis for getting the simulation data:
```python
{data_api}
```

## Constraints:
{constraints}

## Format:
```json
{output_format}
```
## Notes:
- Based on provided APIs, you need to determine whether the constraint's satisfaction can be verified.
- For each constraint, you need to provide a reasoning and a result.
- IF the constraint is satisfied, the result should be True; otherwise, it should be False.
- Is it possible to detect each constraint by writing code?
- The final state data of the robot trajectories can directly serve as the judgment criteria for some constraints.
""".strip()
