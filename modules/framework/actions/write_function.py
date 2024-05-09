import asyncio

from modules.framework.action import ActionNode
from modules.framework.response.code_parser import SingleFunctionParser
from modules.framework.code.function_node import FunctionNode
from modules.framework.response.text_parser import parse_text
from modules.prompt.coding_stage_prompt import WRITE_FUNCTION_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import robot_api
from modules.prompt.task_description import TASK_DES
from modules.prompt.env_description_prompt import ENV_DES
from modules.file.log_file import logger
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.code.function_tree import FunctionTree


class WriteFunction(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self._function = None
        self._constraint_text = ""
        self._other_functions_str = ""
        self._function_pool = FunctionTree()

    def setup(self, function, constraint_text, other_functions_str):
        self._function: FunctionNode = function
        self._constraint_text = constraint_text
        self._other_functions_str = other_functions_str

    def _build_prompt(self):
        self.prompt = WRITE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            env_des=ENV_DES,
            robot_api=robot_api.get_prompt(),
            function_content=self._function._definition,
            constraints=self._constraint_text,
            other_functions=self._other_functions_str,
        )

    def _process_response(self, response: str) -> str:
        desired_function_name = self._function.name
        code = parse_text(text=response)
        parser = SingleFunctionParser()
        parser.parse_code(code)
        parser.check_function_name(desired_function_name)
        self._function_pool.update_from_parser(parser.imports, parser.function_dict)
        return code


class WriteFunctionsAsync(ActionNode):
    def __init__(self, next_text: str, node_name: str = ""):
        super().__init__(next_text, node_name)
        self._function_pool = FunctionTree()

    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        constraint_pool: ConstraintPool = ConstraintPool()

        async def operation(function: FunctionNode):
            logger.log(f"Function: {function.name}", "warning")
            other_functions = self._function_pool.filtered_functions(function)
            other_functions_str = "\n\n".join([f.body for f in other_functions])

            action = WriteFunction()
            action.setup(
                function=function,
                other_functions_str=other_functions_str,
                constraint_text=constraint_pool.filtered_constraints(
                    function.connections
                ),
            )
            return await action.run()

        await self._function_pool.process_function_layers(operation)
        self._function_pool.save_functions_to_file()
