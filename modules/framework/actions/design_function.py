import asyncio
import time

from modules.framework.action import ActionNode, AsyncNode
from modules.framework.code.function_node import FunctionNode, State
from modules.prompt import (
    DesignFunction_PROMPT_TEMPLATE,
    ROBOT_API,
    ENV_DES,
    TASK_DES,
)
from modules.file.log_file import logger
from modules.framework.constraint import ConstraintPool
from modules.framework.code.function_tree import FunctionTree
from modules.framework.parser import SingleFunctionParser, parse_text


class DesignFunction(ActionNode):
    def __init__(self, next_text: str, node_name: str = ""):
        super().__init__(next_text, node_name)
        self._function: FunctionNode = None
        self._function_pool = FunctionTree()

    def setup(self, function):
        self._function = function

    def _build_prompt(self):
        if self._function is None:
            logger.log("Function is not set", "error")
            raise SystemExit

        logger.log(f"Function: {self._function._name}", "warning")

        constraint_pool: ConstraintPool = ConstraintPool()
        other_functions: list[FunctionNode] = self._function_pool.filtered_functions(
            self._function
        )
        other_functions_str = "\n".join([f.brief if not f.body else f.body for f in other_functions])

        self.prompt = DesignFunction_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            function_name=self._function.name,
            function_des=self._function.description,
            constraints=constraint_pool.filtered_constraints(
                related_constraints=self._function.connections
            ),
            other_functions=other_functions_str,
        )

    async def _process_response(self, response: str) -> str:
        desired_function_name = self._function._name
        code = parse_text(text=response)
        parser = SingleFunctionParser()
        parser.parse_code(code)
        parser.check_function_name(desired_function_name)
        new_definition = parser.function_definition
        function_name = parser.function_name
        self._function_pool.set_definition(function_name, new_definition)
        return str(code)


class DesignFunctionAsync(AsyncNode):
    def __init__(self, run_mode='layer', start_state=State.NOT_STARTED, end_state=State.DESIGNED):
        super().__init__(run_mode, start_state, end_state)

    def _build_prompt(self):
        pass

    async def operate(self, function):
        action = DesignFunction("design single function")
        action.setup(function)
        return await action.run()


if __name__ == '__main__':
    import asyncio
    from modules.framework.context import WorkflowContext
    import argparse

    context = WorkflowContext()
    path = "../../../workspace/test"

    function_designer = DesignFunctionAsync('layer')
    function_designer.context.load_from_file(f"{path}/analyze_functions.pkl")
    asyncio.run(function_designer.run())
    context.save_to_file("../../../workspace/test/designed_function.pkl")
