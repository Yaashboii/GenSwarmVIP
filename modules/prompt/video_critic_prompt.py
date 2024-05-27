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
- The output should be in the specified format.
## This is the video frame information:
""".strip()

OUTPUT_TEMPLATE = """
{
    "result": "..."(SUCCESS/FAIL),
    "feedback": "..." (if result is FAIL, you need to provide feedback about the failure reason,and what should be improved)
}""".strip()
