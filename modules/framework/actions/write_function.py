import asyncio

from modules.framework.action import ActionNode
from modules.utils import parse_code, extract_top_level_function_names
from modules.prompt.coding_stage_prompt import WRITE_FUNCTION_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import robot_api
from modules.prompt.task_description import TASK_DES
from modules.prompt.env_description_prompt import ENV_DES


class WriteFunction(ActionNode):
    def __init__(self, next_text: str = '', node_name: str = ''):
        super().__init__(next_text, node_name)
        self._function = None
        self._constraint_text = ''
        self._function_list = []

    def setup(self, function, constraint_text, function_list):
        self._function = function
        self._constraint_text = constraint_text
        self._function_list = function_list

    def _build_prompt(self):
        self.prompt = WRITE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            env_des=ENV_DES,
            robot_api=robot_api.get_prompt(),
            function_content=self._function.definition,
            constraints=self._constraint_text,
            other_functions='\n\n'.join(self._function_list)
        )

    def _process_response(self, response: str) -> str:
        desired_function_name = self._function.name
        code = parse_code(text=response)
        function_list = extract_top_level_function_names(code)
        if not function_list:
            self._context.logger.logger(f"Write Code Failed: No function detected in the response", "error")
            raise Exception
        if len(function_list) > 1:
            self._context.logger.logger(f"Write Code Failed: More than one function detected in the response",
                                        "error")
            raise Exception
        for function_name in function_list:
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                self._context.logger.logger(f"Write Code Failed: No function detected in the response", "error")
                raise Exception
        self._context.function_pool.add_functions(content=code)
        return code

    def _can_skip(self) -> bool:
        return False


class WriteFunctionsAsync(ActionNode):
    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        current_layer_index = 0
        while current_layer_index < len(self._context.function_pool.function_layer):
            tasks = []
            current_layer = self._context.function_pool.function_layer[current_layer_index]
            self._context.logger.log(f"Layer: {current_layer_index}", "warning")
            for function in current_layer:
                self._context.logger.log(f"Function: {function.name}", "warning")
                constraint_text = ''
                for constraint in function.satisfying_constraints:
                    if constraint not in self._context.constraint_pool.constraints:
                        print(f"Constraint {constraint} is not in the constraint pool")
                        raise SystemExit
                    constraint_text += self._context.constraint_pool.constraints[constraint].text + '\n'

                function_list = [f.definition if f.content is None else f.content for f in
                                 self._context.function_pool.functions.values() if f.name != function.name]

                action = WriteFunction()
                action.setup(function=function,
                             function_list=function_list,
                             constraint_text=constraint_text)
                task = asyncio.create_task(action.run())
                tasks.append(task)
            await asyncio.gather(*tasks)
            current_layer = self._context.function_pool.function_layer[current_layer_index]
            # TODO:add logic to rewrite function
            try:
                errors = []
                for function in current_layer:
                    error = self.context.function_pool.check_function_grammar(function_name=function.name)
                    errors.append(error)
            except Exception as e:
                import traceback
                self.context.logger.log(f"error occurred in grammar check:\n {traceback.format_exc()}", 'error')
                raise SystemExit(f"error occurred in async write functions{e}")
            current_layer_index += 1


if __name__ == "__main__":
    from modules.utils import root_manager
    import asyncio

    # for i in range(30):

    path = '../../../workspace/test'
    root_manager.update_root(path, set_data_path=False)

    write_function = WriteFunctionsAsync('design functions async')
    write_function.context.load_from_file(path + "/design_functions.pkl")
    asyncio.run(write_function.run())

    write_function.context.save_to_file(f'{path}/write_functions.pkl')
