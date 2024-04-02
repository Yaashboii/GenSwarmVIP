import asyncio

from modules.framework.action import ActionNode
from modules.utils import parse_code, extract_top_level_function_names
from modules.prompt.coding_stage_prompt import WRITE_FUNCTION_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import robot_api
from modules.prompt.task_description import TASK_DES

class WriteFunction(ActionNode):
    def __init__(self, next_text: str = '', node_name: str = ''):
        super().__init__(next_text, node_name)

    def setup(self, function, constraint_text, function_list):
        self._function = function
        self._constraint_text = constraint_text
        self._function_list = function_list

    def _build_prompt(self):
        self.prompt = WRITE_FUNCTION_PROMPT_TEMPLATE.format(
                    task_des=TASK_DES,
                    robot_api=robot_api.get_prompt(),
                    function_content=self._function.content,
                    constraints=self._constraint_text,
                    other_functions='\n\n'.join(self._function_list)
                )

    def _process_response(self, response: str) -> str:
        desired_function_name = self._function.name
        code = parse_code(text=response)
        function_list = extract_top_level_function_names(code)
        if not function_list:
            self._context.log.format_message(f"Write Code Failed: No function detected in the response", "error")
            raise Exception
        if len(function_list) > 1:
            self._context.log.format_message(f"Write Code Failed: More than one function detected in the response", "error")
            raise Exception
        for function_name in function_list:
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                self._context.log.format_message(f"Write Code Failed: No function detected in the response", "error")
                raise Exception
        self._context.function_pool.add_functions(content=code)
        return code
    
    def _can_skip(self) -> bool:
        return False

class WriteFunctionsAsync(ActionNode):
    def _build_prompt(self):
        return super()._build_prompt()
    
    async def run(self):
        function_layers = self._context.function_pool.function_layer
        for i, layer in enumerate(function_layers):
            tasks = []
            self._context.log.format_message(f"Layer: {i}", "warning")
            for function in layer:
                self._context.log.format_message(f"Function: {function.name}", "warning")
                constraint_text = ''
                for constraint in function.satisfying_constraints:
                    if constraint not in self._context.constraint_pool.constraints:
                        print(f"Constraint {constraint} is not in the constraint pool")
                        raise SystemExit
                    constraint_text += self._context.constraint_pool.constraints[constraint].text + '\n'

                function_list = [f.text if f.content is None else f.content for f in
                                 self._context.function_pool.functions.values() if f.name != function.name]
                
                action = WriteFunction()
                action.setup(function=function, 
                             function_list=function_list, 
                             constraint_text=constraint_text)
                task = asyncio.create_task(action.run())
                tasks.append(task)
            await asyncio.gather(*tasks)

if __name__ == "__main__":
    from modules.utils import root_manager
    import asyncio
    # for i in range(30):

    path = '/src/tests'
    root_manager.update_root(path, set_data_path=False)
    
    write_function = WriteFunctionsAsync('design functions async')
    write_function._context.load_from_file(path + "/design_function_stage.pkl")
    asyncio.run(write_function.run())

    write_function._context.save_to_file(f'{path}/write_function_stage.pkl')