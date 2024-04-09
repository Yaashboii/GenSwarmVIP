from modules.framework.action import ActionNode
from modules.prompt.code_review_stage_prompt import HIGH_LEVEL_FUNCTION_REVIEW
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code, extract_function_definitions, extract_top_level_function_names
import asyncio


class CodeReview(ActionNode):
    def __init__(self, next_text='', node_name=''):
        super().__init__(next_text, node_name)
        self._function = None

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
        self._context.logger.log(f"Reviewing function: {self._function.name}", "warning")

    def _process_response(self, response: str) -> str:
        try:
            desired_function_name = self._function.name
            code = parse_code(text=response)
            function_list = extract_function_definitions(code)
            if not function_list:
                self._context.logger.log(
                    f"High Level Function Review Failed: No function detected in the response",
                    "error")
                return ''
            if len(function_list) > 1:
                self._context.logger.log(
                    f"High Level Function Review Failed: More than one function detected in the response",
                    "error")
                raise Exception(f"More than one function detected in the response")
            function_name = extract_top_level_function_names(code_str=code)[0]
            if function_name != desired_function_name:
                self._context.logger.log(
                    f"High Level Function Review Failed: Function name mismatch: {function_name} != {desired_function_name}",
                    "error")
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            self._context.function_pool.add_functions(content=code)
            for function_name in function_list:
                self._context.function_pool.check_function_grammar(function_name=function_name)
                for function in self._context.function_pool.functions.values():
                    if function_name in function.calls:
                        self._context.function_pool.check_function_grammar(function_name=function.name)
            # # TODO,add bug fix mechanism for such cases,rather than just raising exception to trigger retry
            # if errors:
            #     self._context.logger.log(
            #         f"High Level Function Review Failed: Function {desired_function_name} has syntax error: {errors}",
            #         "error")
            #     raise Exception
            return code
        except ValueError as e:
            self._context.logger.log(f"No function detected in the response: {e}", 'warning')
        except Exception as e:
            self._context.logger.log(f"High Level Function Review Failed: {e}", "error")
            raise Exception  # trigger retry


class CodeReviewAsync(ActionNode):
    def _build_prompt(self):
        return super()._build_prompt()

    async def _run(self):
        current_layer_index = 1
        while current_layer_index < len(self._context.function_pool.function_layer):
            tasks = []
            current_layer = self._context.function_pool.function_layer[current_layer_index]
            self._context.logger.log(f"Layer: {current_layer_index}", "warning")
            for function in current_layer:
                action = CodeReview()
                action.setup(function)
                task = asyncio.create_task(action.run())
                tasks.append(task)
            await asyncio.gather(*tasks)
            layer_index = current_layer_index if current_layer_index < len(
                self._context.function_pool.function_layer) else len(self._context.function_pool.function_layer) - 1
            current_layer = self._context.function_pool.function_layer[layer_index]
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

    path = '../../../workspace/test'
    root_manager.update_root(path, set_data_path=False)

    code_review = CodeReviewAsync("code review")
    code_review.context.load_from_file(path + "/write_run.pkl")
    asyncio.run(code_review.run())

    code_review.context.save_to_file(f'{path}/code_review.pkl')
