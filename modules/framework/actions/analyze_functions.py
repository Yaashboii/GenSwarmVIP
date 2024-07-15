from modules.framework.action import ActionNode
from modules.prompt import (
    ANALYZE_FUNCTION_PROMPT_TEMPLATE,
    FUNCTION_TEMPLATE,
    ROBOT_API,
    ENV_DES,
    TASK_DES,
)
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.code.function_tree import FunctionTree
from modules.file.log_file import logger
from modules.framework.response import *


class AnalyzeFunctions(ActionNode):
    def __init__(self, next_text, node_name=""):
        super().__init__(next_text, node_name)
        self._constraint_pool: ConstraintPool = ConstraintPool()
        self._function_pool = FunctionTree()

    def _build_prompt(self):
        self.prompt = ANALYZE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self.context.command,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            constraints=str(self._constraint_pool),
            output_template=FUNCTION_TEMPLATE,
        )

    async def _process_response(self, response: str) -> str:
        content = parse_text(response, "json")
        self._function_pool.init_functions(content)
        self._constraint_pool.check_constraints_satisfaction()
        logger.log(f"Analyze Functions Success", "success")
        return response


if __name__ == '__main__':
    import asyncio

    function_analyser = AnalyzeFunctions("analyze constraints")
    path = "../../../workspace/test"
    function_analyser.context.load_from_file(f"{path}/constraint.pkl")
    asyncio.run(function_analyser.run())
    function_analyser.context.save_to_file(f"{path}/analyze_functions.pkl")
