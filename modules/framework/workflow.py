import asyncio

from modules.stages import *
from modules.actions import *
from modules.stages.stage import StageType, StageResult
from modules.utils.logger import setup_logger
from modules.framework.stage_transition import StageTransition
from modules.framework.workflow_context import WorkflowContext


class Workflow:
    STAGE_POOL = {
        StageType.AnalyzeStage: AnalysisStage(AnalyzeConstraints()),
        StageType.DesignStage: DesignStage(DesignFunction()),
        StageType.CodingStage: CodingStage(WriteCode()),
        StageType.RunningStage: RunningStage(RunCode()),
        StageType.ReviewStage: ReviewStage(),
        StageType.FinalStage: FinalStage(),
    }

    # ACTION_POOL = {
    #     ActionType.WriteCode: WriteCode(),
    #     ActionType.RewriteCode: RewriteCode(),
    #     ActionType.WritePrompt: WritePrompt(),
    #     ActionType.RunCode: RunCode(),
    # }

    def __init__(self, user_command: str, init_stage: StageType = StageType.AnalyzeStage, args=None):
        self.__stage = init_stage
        self._logger = setup_logger("Workflow")
        self._context = WorkflowContext()
        # workflow_context = WorkflowContext()
        # workflow_context.user_command.message = user_command
        # workflow_context.args = args
        self._context.user_command.message = user_command
        self._context.args = args
    async def run(self):
        while self.__stage != StageType.FinalStage:
            stage = self.create_stage(self.__stage)
            stage_result = await stage.run()

            temp = StageTransition[self.__stage]
            for key in stage_result.keys:
                temp = temp[key]

            self.__stage = temp
        else:
            self._context.log.format_message("=========END=========", "success")

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
        "Gather these robots together at the position of leader robot",
        "Gather the robots to point (2,3)",
        "Gather the robots along the y=x trajectory",
        'Move the robots to form a square formation',
        'First, move the robot to form a square formation. Then, move the robots to form a triangle formation.Finally gather these robots together',
        "Initially, gather all robots at the center of the environment, confirming their arrival before proceeding. Next, arrange the robots into a square formation with each side measuring exactly 1.0 meter, ensuring the formation's precision with right angles and equal sides. Once the square is confirmed, guide the robots to trace a circular path while maintaining the square formation. Constant monitoring is required to preserve the formation's integrity and the path's accuracy throughout the movement."
    ]
    workflow = Workflow(task_list[0])
    asyncio.run(workflow.run())
