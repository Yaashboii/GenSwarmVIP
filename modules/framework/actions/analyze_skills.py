from sympy.printing.dot import template

from modules.file import logger
from modules.framework.action import ActionNode
from modules.framework.constraint import ConstraintPool
from modules.framework.code import FunctionTree
from modules.framework.parser import *
from modules.prompt import (
    ANALYZE_SKILL_PROMPT_TEMPLATE,
    FUNCTION_TEMPLATE,
    GLOBAL_ROBOT_API,
    LOCAL_ROBOT_API,
    ALLOCATOR_TEMPLATE,
    ENV_DES,
    TASK_DES,
)
from modules.utils import root_manager


class AnalyzeSkills(ActionNode):
    def __init__(self, next_text, node_name=""):
        super().__init__(next_text, node_name)
        self._constraint_pool: ConstraintPool = self.context.constraint_pool

    def _build_prompt(self):
        self.prompt = self.prompt.format(
            task_des=TASK_DES,
            instruction=self.context.command,
            local_api=LOCAL_ROBOT_API + ALLOCATOR_TEMPLATE.format(template='Temporarily unknown'),
            global_api=GLOBAL_ROBOT_API,
            env_des=ENV_DES,
            constraints=str(self._constraint_pool),
            output_template=FUNCTION_TEMPLATE,
        )

    async def _process_response(self, response: str) -> str:
        content = parse_text(response, "json")
        functions = eval(content)["functions"]
        global_functions = []
        local_functions = []
        for function in functions:
            if function["scope"] == "global":
                global_functions.append(function)
            else:
                local_functions.append(function)
        self.context.global_skill_tree.init_functions(global_functions)
        self.context.local_skill_tree.init_functions(local_functions)
        if len(global_functions) == 0:
            logger.log("No global functions detected,generate local skills", "warning")
            self.context.scoop = 'local'
            from modules.framework.actions import GenerateFunctions
            self._next = GenerateFunctions()

        self._constraint_pool.check_constraints_satisfaction()
        logger.log(f"Analyze Functions Success", "success")
        return response


if __name__ == '__main__':
    import asyncio

    function_analyser = AnalyzeSkills("analyze constraints")
    path = "../../../workspace/test"
    root_manager.update_root('../../../workspace/test')

    function_analyser.context.load_from_file(f"{path}/constraint.pkl")
    asyncio.run(function_analyser.run())
    function_analyser.context.save_to_file(f"{path}/analyze_functions.pkl")
