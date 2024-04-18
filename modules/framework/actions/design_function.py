import asyncio

from modules.framework.action import ActionNode
from modules.utils import parse_code, extract_function_definitions, extract_top_level_function_names
from modules.prompt.design_stage_prompt import DesignFunction_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import robot_api
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.framework.context import logger, ConstraintPool

class DesignFunction(ActionNode):
    def __init__(self, next_text: str, node_name: str = ''):
        super().__init__(next_text, node_name)
        self._function = None

    def setup(self, function):
        self._function = function

    def _check_function(self):
        if self._function is None:
            logger.log("Function is not set", "error")
            raise SystemExit
        logger.log(f"Function: {self._function.name}", "warning")

    def _build_prompt(self):
        self._check_function()
        constraint_pool : ConstraintPool = ConstraintPool()

        function_list = [f.text if f.definition is None else f.definition for f in
                         self.context.functions_value if f.name != self._function.name]

        self.prompt = DesignFunction_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            robot_api=robot_api.get_prompt(),
            env_des=ENV_DES,
            function_name=self._function.name,
            function_des=self._function.description,
            constraints=constraint_pool.filtered_constaints(keys=self._function.satisfying_constraints),
            other_functions='\n'.join(function_list)
        )

    def _process_response(self, response: str) -> str:
        desired_function_name = self._function.name
        code = parse_code(text=response)
        function_list = extract_function_definitions(code)
        if not function_list:
            logger.log(f"Design Function Failed: No function detected in the response", "error")
            raise Exception  # trigger retry
        if len(function_list) > 1:
            logger.log(f"Design Function Failed: More than one function detected in the response",
                                     "error")
            raise Exception  # trigger retry
        for function in function_list:
            function_name = extract_top_level_function_names(code_str=function)[0]
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                logger.log(f"Design Function Failed: No function detected in the response",
                                         "error")
                raise Exception  # trigger retry
            self.context.set_function_definition(function_name=function_name, definition=function)
        return str(code)

class DesignFunctionAsync(ActionNode):
    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        function_layers = self.context.function_layer
        if not function_layers:
            logger.log("No function to design", "error")
            raise SystemExit
        for i, layer in enumerate(function_layers):
            tasks = []
            logger.log(f"Layer: {i}", "warning")
            for function in layer:
                action = DesignFunction('design single function')
                action.setup(function)
                task = asyncio.create_task(action.run())
                tasks.append(task)
            await asyncio.gather(*tasks)


if __name__ == "__main__":
    from modules.utils import root_manager

    path = '../../../workspace/test'
    root_manager.update_root(path)

    design_functions = DesignFunctionAsync('design functions async')
    design_functions.context.load_from_file(path + "/analyze_functions.pkl")
    asyncio.run(design_functions.run())

    design_functions.context.save_to_file(f'{path}/design_functions.pkl')
