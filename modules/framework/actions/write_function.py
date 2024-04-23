import asyncio

from modules.framework.action import ActionNode
from modules.framework.code.code import AstParser
from modules.framework.context.node import FunctionNode
from modules.framework.code.code import parse_code
from modules.prompt.coding_stage_prompt import WRITE_FUNCTION_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import robot_api
from modules.prompt.task_description import TASK_DES
from modules.prompt.env_description_prompt import ENV_DES
from modules.file.log_file import logger
from modules.framework.context import ConstraintPool, FunctionPool


class WriteFunction(ActionNode):
    def __init__(self, next_text: str = '', node_name: str = ''):
        super().__init__(next_text, node_name)
        self._function = None
        self._constraint_text = ''
        self._other_functions = []
        self._function_pool = FunctionPool()


    def setup(self, function, constraint_text, other_functions):
        self._function = function
        self._constraint_text = constraint_text
        self._other_functions = other_functions

    def _build_prompt(self):
        self.prompt = WRITE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            env_des=ENV_DES,
            robot_api=robot_api.get_prompt(),
            function_content=self._function.definition,
            constraints=self._constraint_text,
            other_functions='\n\n'.join(self._other_functions)
        )

    def _process_response(self, response: str) -> str:
        desired_function_name = self._function.name
        code = parse_code(text=response)
        code_obj = AstParser(code)
        function_list = code_obj.function_names
        if not function_list:
            logger.log(f"Write Code Failed: No function detected in the response", "error")
            raise Exception
        if len(function_list) > 1:
            logger.log(f"Write Code Failed: More than one function detected in the response",
                                     "error")
            raise Exception
        for function_name in function_list:
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                logger.log(f"Write Code Failed: No function detected in the response", "error")
                raise Exception
        self._function_pool.add_functions(content=code)
        return code


class WriteFunctionsAsync(ActionNode):
    def __init__(self, next_text: str, node_name: str = ''):
        super().__init__(next_text, node_name)
        self._function_pool = FunctionPool()

    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        constraint_pool : ConstraintPool = ConstraintPool()
        async def operation(function: FunctionNode):
            logger.log(f"Function: {function.name}", "warning")
            other_functions : list[FunctionNode] = self._function_pool.filtered_functions(function)
            other_functions_str = '\n\n'.join([f.content for f in other_functions])

            action = WriteFunction()
            action.setup(function=function,
                         other_functions=other_functions_str,
                         constraint_text=constraint_pool.filtered_constaints(function.connections))
            return await action.run()
        self._function_pool.process_function_layers(operation, start_layer_index=1)      
         


if __name__ == "__main__":
    from modules.utils import root_manager
    import asyncio

    # for i in range(30):

    path = '../../../workspace/test'
    root_manager.update_root(path)

    write_function = WriteFunctionsAsync('design functions async')
    write_function.context.load_from_file(path + "/design_functions.pkl")
    asyncio.run(write_function.run())

    write_function.context.save_to_file(f'{path}/write_functions.pkl')
