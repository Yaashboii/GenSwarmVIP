from modules.framework.action import ActionNode
from modules.framework.context import WorkflowContext
from .base_alation import BaseAlation
from modules.framework.actions import Criticize


class HumanFeedbackAlation(BaseAlation):
    def __init__(self,
                 workspace_name: str,
                 exp_list: list):
        criticizer = Criticize()
        super().__init__(criticizer, workspace_name, exp_list)

    def setup(self, directory: str):
        self.tester = Criticize()
        self.tester.context.load_from_file(f"{directory}/WriteRun.pkl")
        feedback_file_path = f"{directory}/human_feedback.txt"
        with open(feedback_file_path, 'r') as file:
            feedback = file.read()
        self.tester.setup(feedback)

    async def run_single(self, directory: str):
        await self.tester.run()
        self.tester.context.save_to_file(f"{directory}/human_feedback.pkl")
