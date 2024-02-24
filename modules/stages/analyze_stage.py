from modules.stages.stage import Stage, StageResult
from modules.actions import AnalyzeReqs
from modules.const import ROBOT_API, ENV_DES

PROMPT_TEMPLATE: str = """
To assist users in automating specific tasks,and can automatically exit after task completion,you need to analyze the sub-functions required to complete the task based on the user's simple task instructions, considering the resources available in the current environment.
You also need to consider any constraints. 
{env_des}
user requirements:{instruction}
APIs: {api}
Output the analysis in the following format:

User Requirement Description: Detailed description of the task the user hopes to automate.
Functional Requirements:
List the functions expected to be developed based on user needs and available resources.
For each function, briefly describe its purpose and expected outcome.
These functions should be decoupled from each other.
Task Constraints:
Detail any specific requirements or restrictions in terms of performance, security, etc."
""".strip()


class AnalyzeStage(Stage):
    def __init__(self, action: AnalyzeReqs):
        super(AnalyzeStage, self).__init__()
        self._user_command = ""
        self._action = action

    async def _run(self) -> StageResult:
        prompt = PROMPT_TEMPLATE.format(instruction=self._context.user_command, api=ROBOT_API, env_des=ENV_DES)
        await self._action.run(prompt=prompt)
        return StageResult(keys=[])


if __name__ == "__main__":
    stage = AnalyzeStage(AnalyzeReqs())
    res = stage.run()
    print(res)
