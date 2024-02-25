import asyncio

from modules.stages import *
from modules.actions import *
from modules.stages.stage import StageType, StageResult
from modules.utils.logger import setup_logger
from modules.framework.stage_transition import StageTransition
from modules.framework.workflow_context import WorkflowContext


class Workflow:
    STAGE_POOL = {
        StageType.AnalyzeStage: AnalyzeStage(AnalyzeReqs()),
        StageType.DesignStage: DesignStage(DesignFunction()),
        StageType.CodingStage: CodingStage(WriteCode()),
        StageType.RunningStage: RunningStage(RunCode()),
        StageType.FinalStage: FinalStage(),
    }

    # ACTION_POOL = {
    #     ActionType.WriteCode: WriteCode(),
    #     ActionType.RewriteCode: RewriteCode(),
    #     ActionType.WritePrompt: WritePrompt(),
    #     ActionType.RunCode: RunCode(),
    # }

    def __init__(self, user_command: str, init_stage: StageType = StageType.AnalyzeStage):
        self.__stage = init_stage
        self._logger = setup_logger("Workflow")
        workflow_context = WorkflowContext()
        workflow_context.user_command = user_command

    async def run(self):
        while self.__stage != StageType.FinalStage:
            stage = self.create_stage(self.__stage)
            stage_result = await stage.run()

            temp = StageTransition[self.__stage]
            for key in stage_result.keys:
                temp = temp[key]

            self.__stage = temp
        else:
            self._logger.info("=================== END ===================")

    @staticmethod
    def create_stage(stage_type: StageType):
        try:
            return Workflow.STAGE_POOL[stage_type]
        except KeyError:
            raise ValueError(f"Invalid stage type: {stage_type}")

    # @staticmethod
    # def create_action(action_type: ActionType):
    #     if action_type == ActionType.WriteCode:
    #         return Workflow.ACTION_POOL[ActionType.WriteCode]
    #     elif action_type == ActionType.RewriteCode:
    #         return Workflow.ACTION_POOL[ActionType.RewriteCode]
    #     elif action_type == ActionType.WritePrompt:
    #         return Workflow.ACTION_POOL[ActionType.WritePrompt]
    #     elif action_type == ActionType.RunCode:
    #         return Workflow.ACTION_POOL[ActionType.RunCode]
    #     else:
    #         raise ValueError("Invalid action type")


if __name__ == "__main__":
    task_list = [
        "Gather these robots together",
        'Move the robots to form a hexagon',
        'All the robots form a rectangular formation and move within the environment.',
    ]
    workflow = Workflow(task_list[2])
    asyncio.run(workflow.run())
