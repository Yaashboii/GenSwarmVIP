import asyncio

from modules.framework.actions import *
from modules.framework.action import *
from modules.framework.handler import *

from modules.utils.logger import setup_logger
from modules.framework.workflow_context import WorkflowContext

class Workflow:
    def __init__(self, user_command: str, args=None):
        self._logger = setup_logger("Workflow")
        self._context = WorkflowContext()
        self._context.user_command.message = user_command
        self._context.args = args
        self._pipeline = None

        self.build_up()

    def build_up(self):
        # initialize actions
        analyze_requirements = AnalyzeReqs('analysis')
        design_function = DesignFunction("functions definition")
        run_code = RunCode("pass")
        write_functions = WriteFunctions("function.py")
        write_run = WriteRun("run.py")
        write_seq_diagram = WriteSeqDiagram("Sequence diagram")
        critic_check_1 = CriticCheck("pass", node_name="Critic_1")
        critic_check_2 = CriticCheck("pass", node_name="Critic_2")

        # initialize error handlers
        bug_handler = BugLevelHandler()
        critic_handler = CriticLevelHandler()
        hf_handler = HumanFeedbackHandler()
        self._chain_of_handler = critic_handler
        critic_handler.successor = bug_handler
        bug_handler.successor = hf_handler
        
        # link actions
        ## stage 1
        stage_code_generation = ActionLinkedList("Code Generation", analyze_requirements)
        stage_code_generation.add(design_function)
        stage_code_generation.add(write_functions)
        stage_code_generation.add(write_seq_diagram)
        stage_code_generation.add(write_run)
        ## stage 2
        stage_critic_check = ActionLinkedList("Critic Check", critic_check_1)
        stage_critic_check.add(critic_check_2)
        ## stage 3
        stage_code_execution = ActionLinkedList("Code Execution", run_code)
        # combine stages
        code_llm = ActionLinkedList("Code-LLM", stage_code_generation)
        code_llm.add(stage_critic_check)
        code_llm.add(stage_code_execution)
        code_llm.add(ActionNode("","END"))
        self._pipeline = code_llm
        # assign error handlers to actions
        critic_check_1.error_handler = self._chain_of_handler
        critic_check_2.error_handler = self._chain_of_handler
        run_code.error_handler = self._chain_of_handler
        critic_handler.next_action= stage_code_generation


    async def run(self):
        text = display_all(self._pipeline, self._chain_of_handler)
        from modules.framework.workflow_context import FileInfo
        flow = FileInfo(name='flow.md')
        flow.message = text
        

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
