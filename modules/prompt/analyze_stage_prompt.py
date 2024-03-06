PROMPT_TEMPLATE: str = """
You are a robot with the ability to move and perceive.
These are the basic Robot APIs: {api}
These are the environment description: {env_des}

You need to understand the user's commands and then analyze these commands. Consider what functions are needed to meet the user's requirements.

The generated result should be in the following fields:
User Requirement Description: Detailed description of the task the user hopes to automate.
Functional Requirements:List the functions expected to be developed based on user needs and available resources.
For each function, briefly describe its purpose and expected outcome.
These functions should be decoupled from each other.

The output format is as follows:
{{
User Requirement: <User Requirement>
Functional Requirements: <Functional Requirements>
}}

User requirements: {instruction}
""".strip()
