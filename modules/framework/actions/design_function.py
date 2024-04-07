import asyncio

from modules.framework.action import ActionNode
from modules.utils import parse_code, extract_function_definitions, extract_top_level_function_names
from modules.prompt.design_stage_prompt import DesignFunction_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import robot_api
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES


class DesignFunction(ActionNode):
    def __init__(self, next_text: str, node_name: str = ''):
        super().__init__(next_text, node_name)
        self._function = None

    def setup(self, function):
        self._function = function

    def _build_prompt(self):
        if self._function is None:
            self._context.logger.log("Function is not set", "error")
            raise SystemExit
        self._context.logger.log(f"Function: {self._function.name}", "warning")
        constraint_text = ''
        for constraint in self._function.satisfying_constraints:
            if constraint not in self._context.constraint_pool.constraints:
                self.context.logger.log(f"Constraint {constraint} is not in the constraint pool", 'error')
                raise SystemExit
            constraint_text += self._context.constraint_pool.constraints[constraint].text + '\n'

        function_list = [f.text if f.definition is None else f.definition for f in
                         self._context.function_pool.functions.values() if f.name != self._function.name]

        self.prompt = DesignFunction_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            robot_api=robot_api.get_prompt(),
            env_des=ENV_DES,
            function_name=self._function.name,
            function_des=self._function.description,
            constraints=constraint_text,
            other_functions='\n'.join(function_list)
        )

    def _process_response(self, response: str) -> str:
        desired_function_name = self._function.name
        code = parse_code(text=response)
        function_list = extract_function_definitions(code)
        if not function_list:
            self._context.logger.log(f"Design Function Failed: No function detected in the response", "error")
            raise Exception  # trigger retry
        if len(function_list) > 1:
            self._context.logger.log(f"Design Function Failed: More than one function detected in the response",
                                     "error")
            raise Exception  # trigger retry
        for function in function_list:
            function_name = extract_top_level_function_names(code_str=function)[0]
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                self._context.logger.log(f"Design Function Failed: No function detected in the response",
                                         "error")
                raise Exception  # trigger retry
            self._context.function_pool.functions[function_name].definition = function
        return str(code)

    def _can_skip(self) -> bool:
        # TODO: can skip when functions are all right
        return False


class DesignFunctionAsync(ActionNode):
    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        function_layers = self._context.function_pool.function_layer
        if not function_layers:
            self._context.logger.log("No function to design", "error")
            raise SystemExit
        for i, layer in enumerate(function_layers):
            tasks = []
            self._context.logger.log(f"Layer: {i}", "warning")
            for function in layer:
                action = DesignFunction('design single function')
                action.setup(function)
                task = asyncio.create_task(action.run())
                tasks.append(task)
            await asyncio.gather(*tasks)

        pass


if __name__ == "__main__":
    from modules.utils import root_manager

    path = '../../../workspace/test'
    root_manager.update_root(path, set_data_path=False)

    design_functions = DesignFunctionAsync('design functions async')
    design_functions.context.load_from_file(path + "/analyze_functions.pkl")
    asyncio.run(design_functions.run())

    design_functions.context.save_to_file(f'{path}/design_functions.pkl')
