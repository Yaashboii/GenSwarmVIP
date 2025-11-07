from modules.framework.action import ActionNode, ActionLinkedList
from modules.framework.actions import AnalyzeConstraints, AnalyzeSkills, GenerateFunctions, RunCode, RunCodeAsync, DebugError, CodeImprove, VideoCriticize
from modules.framework.workflow import Workflow
from modules.utils import root_manager, rich_print
from modules.prompt.user_requirements import get_user_commands

import asyncio
from modules.framework.context import WorkflowContext
import argparse

root_manager.update_root("../../../workspace/test")

parser = argparse.ArgumentParser(
    description="Run simulation with custom parameters."
)

parser.add_argument(
    "--interaction_mode",
    type=bool,
    default=False,
    help="Whether to run in interaction mode in analyze constraints.",
)

parser = argparse.ArgumentParser()
parser.add_argument('--llm_name', type=str, default='tngtech/deepseek-r1t-chimera:free', help='Name of the language model to use')

context = WorkflowContext()
task = get_user_commands("covering")[0]

context.command = task
args = parser.parse_args()
context.args = args
constraint_analyser = AnalyzeConstraints("analyze constraints")

asyncio.run(constraint_analyser.run())
context.save_to_file("../../../workspace/test/constraint.pkl")


