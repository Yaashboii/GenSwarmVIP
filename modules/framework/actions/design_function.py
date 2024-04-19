import asyncio

from modules.framework.action import ActionNode
from modules.framework.code.code import Code
from modules.framework.context.node import FunctionNode
from modules.utils.common import parse_code
from modules.prompt.design_stage_prompt import DesignFunction_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.file.log_file import logger
from modules.framework.context import ConstraintPool, FunctionPool

class DesignFunction(ActionNode):
    def __init__(self, next_text: str, node_name: str = ''):
        super().__init__(next_text, node_name)
        self._function : FunctionNode = None
        self._function_pool = FunctionPool()

    def setup(self, function):
        self._function = function

    def _build_prompt(self):
        if self._function is None:
            logger.log("Function is not set", "error")
            raise SystemExit
        
        logger.log(f"Function: {self._function._name}", "warning")

        constraint_pool : ConstraintPool = ConstraintPool()
        function_list = self._function_pool.filtered_function_info(self._function)

        self.prompt = DesignFunction_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            function_name=self._function._name,
            function_des=self._function._description,
            constraints=constraint_pool.filtered_constaints(keys=self._function._satisfying_constraints),
            other_functions='\n'.join(function_list)
        )

    def _process_response(self, response: str) -> str:
        def check_error(function_list):
            if not function_list:
                logger.log(f"Design Function Failed: No function detected in the response", "error")
                raise Exception  # trigger retry
            if len(function_list) > 1:
                logger.log(f"Design Function Failed: More than one function detected in the response",
                                        "error")
                raise Exception  # trigger retry
        def check_function(function_name, desired_function_name):
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                logger.log(f"Design Function Failed: No function detected in the response",
                                         "error")
                raise Exception  # trigger retry
        desired_function_name = self._function._name
        code = parse_code(text=response)
        code_obj = Code(code)
        definition_list = code_obj.extract_function_definitions()
        check_error(definition_list)

        for definition in definition_list:
            code_obj = Code(definition)
            function_name = code_obj.extract_top_level_function_names()[0]
            check_function(function_name, desired_function_name)
            self._function_pool.set_definiton(function_name, definition)
        return str(code)

class DesignFunctionAsync(ActionNode):
    def _build_prompt(self):
        pass

    async def _run(self):
        function_pool = FunctionPool()
        async def operation(function: FunctionNode):
            action = DesignFunction('design single function')
            action.setup(function)
            return await action.run()
        function_pool.async_handle_function_by_layer(operation, start_layer_index=0, check_grammer=False)   


if __name__ == "__main__":
    from modules.utils import root_manager

    path = '../../../workspace/test'
    root_manager.update_root(path)

    design_functions = DesignFunctionAsync('design functions async')
    design_functions.context.load_from_file(path + "/analyze_functions.pkl")
    asyncio.run(design_functions.run())

    design_functions.context.save_to_file(f'{path}/design_functions.pkl')
