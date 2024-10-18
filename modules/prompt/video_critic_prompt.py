"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""

VIDEO_PROMPT_TEMPLATE = """
## Background:
{task_des}

## Role setting:
- You need to determine whether the task has been successfully completed by analyzing the video after it has been processed.

## This is the user requirement information:
{command}

## This is the constraint information:
{constraint}

## This is the feedback from the user:
{feedback}

## Output format:
"reasoning": "..."(Think step by step and analyze the video frame information, user requirement information, and constraint information. Determine whether the task has been successfully completed.),
```json
{out_put}
```

## Notes:
- You need to determine whether the task has been successfully completed based on the user's requirements and the analyzed constraints.
- You need to reason step by step and consider carefully before drawing a conclusion.
-The user's feedback may be inconsistent with the initial requirements, and even the feedback itself may be inconsistent. In case of any conflict, the most recent feedback should take precedence.
- For certain requirements where the details are difficult to define, such as precise numerical values, the criteria can be appropriately relaxed to allow for approximate values.
- Green represents the robot, gray represents obstacles, and an overlap indicates a collision.
-For constraints that can be assessed through video, judgments will be made. Constraints that cannot be determined through video will not be judged. For example, exact numerical values will not be assessed, but qualitative collision detection will be.
- The output should be in the specified format.
## This is the video frame information:
""".strip()

OUTPUT_TEMPLATE = """
{
    "result": "..."(SUCCESS/FAIL),
    "feedback": "..." (if result is FAIL, you need to provide feedback about the failure reason,and what should be improved)
}""".strip()
