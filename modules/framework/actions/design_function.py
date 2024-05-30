import asyncio

from modules.framework.action import ActionNode
from modules.framework.response.code_parser import SingleFunctionParser
from modules.framework.code.function_node import FunctionNode
from modules.framework.response.text_parser import parse_text
from modules.prompt.design_stage_prompt import DesignFunction_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.file.log_file import logger
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.code.function_tree import FunctionTree


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
        other_functions_str = "\n".join([f.brief for f in other_functions])

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

    def _process_response(self, response: str) -> str:
        desired_function_name = self._function._name
        code = parse_text(text=response)
        parser = SingleFunctionParser()
        parser.parse_code(code)
        parser.check_function_name(desired_function_name)
        new_definition = parser.function_definition
        function_name = parser.function_name
        self._function_pool.set_definition(function_name, new_definition)
        # logger.log(f"new definition: {new_definition}")
        return str(code)

    async def operate_on_node(self, function_node: FunctionNode):
        self._function = function_node
        return await self.run()
