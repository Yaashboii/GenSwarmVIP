from modules.stages.stage import Stage, StageResult
from modules.actions import AnalyzeReqs
from modules.const import ROBOT_API, ENV_DES

PROMPT_TEMPLATE: str = """
There are some ground-moving robots in the room, and users will issue commands to direct their movement. 
You need to understand the user's commands and then analyze these commands. Consider what functions are needed to meet the user's requirements.
{env_des}
user requirements:{instruction}
APIs: {api}
Output the analysis in the following format:

User Requirement Description: Detailed description of the task the user hopes to automate.
Functional Requirements:
List the functions expected to be developed based on user needs and available resources.
For each function, briefly describe its purpose and expected outcome.
These functions should be decoupled from each other.
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
