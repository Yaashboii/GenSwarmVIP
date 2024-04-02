import asyncio

from modules.framework.action import ActionNode
from modules.utils import parse_code

from modules.prompt.code_review_stage_prompt import HIGH_LEVEL_FUNCTION_REVIEW
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code, extract_function_definitions, extract_top_level_function_names

class CodeReview(ActionNode):
    def __init__(self, next_text = '', node_name = ''):
        super().__init__(next_text, node_name)

    def _build_prompt(self):
        self.prompt = HIGH_LEVEL_FUNCTION_REVIEW.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            function_name=self._function.name,
            other_functions='\n\n'.join(
                ['\n'.join(f.import_list) + f.content
                  for f in self._context.function_pool.functions.values() if
                  f.name != self._function.name]),
            function_content=self._function.content,
        )

    def setup(self, function):
        self._function = function

    def _process_response(self, response: str) -> str:
        try:
            desired_function_name = self._function.name
            code = parse_code(text=response)
            function_list = extract_function_definitions(code)
            if not function_list:
                self._context.log.format_message(
                    f"High Level Function Review Failed: No function detected in the response",
                    "error")
                return ''
            if len(function_list) > 1:
                self._context.log.format_message(
                    f"High Level Function Review Failed: More than one function detected in the response",
                    "error")
                raise Exception(f"More than one function detected in the response")
            function_name = extract_top_level_function_names(code_str=code)[0]
            if function_name != desired_function_name:
                self._context.log.format_message(
                    f"High Level Function Review Failed: Function name mismatch: {function_name} != {desired_function_name}",
                    "error")
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            self._context.function_pool.add_functions(content=code)
            return code
        except ValueError as e:
            self._context.log.format_message(f"No function detected in the response: {e}", 'warning')
        except Exception as e:
            self._context.log.format_message(f"High Level Function Review Failed: {e}", "error")
            raise Exception  # trigger retry
    
    
class CodeReviewAsync(ActionNode):
    def _build_prompt(self):
        return super()._build_prompt()

    async def run(self):
        function_layers = self._context.function_pool.function_layer
        for i, layer in enumerate(function_layers[1:]):
            tasks = []
            self._context.log.format_message(f"Layer: {i + 1}", "warning")
            for function in layer:
                action = CodeReview()
                action.setup(function)
                task = asyncio.create_task(action.run())
                tasks.append(task)
            await asyncio.gather(*tasks)

        
if __name__ == "__main__":
    from modules.utils import root_manager
    import asyncio
    # for i in range(30):

    path = '/src/tests'
    root_manager.update_root(path, set_data_path=False)
    
    code_review = CodeReviewAsync("code review")
    code_review._context.load_from_file(path + "/write_run_stage.pkl")
    asyncio.run(code_review.run())

    code_review._context.save_to_file(f'{path}/code_review.pkl')
    
