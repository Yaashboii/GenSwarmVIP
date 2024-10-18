"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""

from modules.framework.action import ActionNode
from modules.framework.code import FunctionTree
from modules.framework.parser import CodeParser, parse_text
from modules.llm import GPT
from modules.prompt import (
    FEEDBACK_PROMPT_TEMPLATE,
    CONTINUE_FEEDBACK_PROMPT_TEMPLATE,
    LOCAL_ROBOT_API,
    GLOBAL_ROBOT_API,
    ENV_DES,
    TASK_DES,
)


class Criticize(ActionNode):
    def __init__(self, skill_tree, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self.__llm = GPT(memorize=True)
        self.feedback = None
        self._skill_tree = skill_tree
        self._call_times = 0

    def _build_prompt(self):
        if self._skill_tree.name == "global_skill":
            function_scoop_note = (
                "This function is executed on a global central controller, considering the information of all robots and the task objectives."
                "It uses the optimal and efficient algorithm to coordinate tasks among multiple robots."
            )
            robot_api_str = GLOBAL_ROBOT_API
        else:
            function_scoop_note = "This function runs on the robot itself, using the provided perception API and motion API to execute its own tasks."
            robot_api_str = LOCAL_ROBOT_API
        if self._call_times == 0:
            self.prompt = FEEDBACK_PROMPT_TEMPLATE.format(
                task_des=TASK_DES,
                robot_api=robot_api_str,
                env_des=ENV_DES,
                functions="\n\n\n".join(self._skill_tree.functions_body),
                feedback=self.feedback,
            )
        else:
            self.prompt = CONTINUE_FEEDBACK_PROMPT_TEMPLATE.format(
                feedback=self.feedback,
            )
        self._call_times += 1

    def setup(self, feedback: str):
        self.feedback = feedback

    async def _process_response(self, response: str, **kwargs) -> str:
        code = parse_text(text=response)
        parser = CodeParser()
        parser.parse_code(code)
        function_list = parser.function_names
        self._skill_tree.update_from_parser(parser.imports, parser.function_dict)
        self._skill_tree.save_functions_to_file()
        return str(code)


if __name__ == "__main__":
    critic = Criticize("constraints")
    from modules.utils import root_manager
    import asyncio

    path = "../../../workspace/test"
    root_manager.update_root(path)
    critic.context.load_from_file(path + "/run_code.pkl")
    asyncio.run(critic.run())
    critic.context.save_to_file(f"{path}/analyze_constraints.pkl")
