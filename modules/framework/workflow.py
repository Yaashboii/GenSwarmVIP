from modules.stages import *
from modules.actions import *
from modules.stages.stage import StageType, StageResult
from modules.utils.logger import setup_logger
from modules.framework.stage_transition import StageTransition
from modules.framework.workflow_context import WorkflowContext

class Workflow:
    STAGE_POOL = {
        StageType.AnalyzeStage: AnalyzeStage(WritePrompt()),
        StageType.DesignStage: DesignStage(WriteDesign()),
        StageType.TestingStage: TestStage(WriteCode()),
        StageType.CodingStage: CodingStage(WriteCode()),
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

    def run(self):
        while self.__stage != StageType.FinalStage:
            stage = self.create_stage(self.__stage)
            stage_result = stage.run()

            temp = StageTransition[self.__stage]
            for key in stage_result.keys:
                temp = temp[key]
                
            self.__stage = temp
        else:
            self._logger.info("=================== END ===================")

    @staticmethod
    def create_stage(stage_type: StageType):
        if stage_type == StageType.AnalyzeStage:
            return Workflow.STAGE_POOL[StageType.AnalyzeStage]
        elif stage_type == StageType.DesignStage:
            return Workflow.STAGE_POOL[StageType.DesignStage]
        elif stage_type == StageType.CodingStage:
            return Workflow.STAGE_POOL[StageType.CodingStage]
        elif stage_type == StageType.TestingStage:
            return Workflow.STAGE_POOL[StageType.TestingStage]
        elif stage_type == StageType.FinalStage:
            return Workflow.STAGE_POOL[StageType.FinalStage]
        else:
            raise ValueError("Invalid stage type")
        
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
    workflow = Workflow("move in circle")
    workflow.run()