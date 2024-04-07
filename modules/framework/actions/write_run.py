from modules.framework.action import ActionNode
from modules.utils import parse_code, extract_imports_and_functions, extract_top_level_function_names
from modules.prompt.coding_stage_prompt import WRITE_RUN_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES


class WriteRun(ActionNode):
    def _build_prompt(self):
        functions = '\n\n'.join(
            [f.content for f in self._context.function_pool.functions.values() if f.content is not None])
        self.prompt = WRITE_RUN_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            env_des=ENV_DES,
            robot_api=ROBOT_API,
            functions=functions,
        )

    def _process_response(self, response: str) -> str:
        desired_function_name = "run_loop"
        code = parse_code(text=response)
        function_list = extract_top_level_function_names(code)
        if not function_list:
            self._context.logger.log(f"Write Code Failed: No function detected in the response", "error")
            raise Exception
        if len(function_list) > 1:
            self._context.logger.log(f"Write Code Failed: More than one function detected in the response", "error")
            raise Exception
        for function_name in function_list:
            if function_name != desired_function_name:
                raise Exception(f"Function name mismatch: {function_name} != {desired_function_name}")
            if not function_name:
                self._context.logger.log(f"Write Code Failed: No function detected in the response", "error")
                raise Exception
        self._context.function_pool.add_functions(content=code)
        error = self.context.function_pool.check_function_grammar(function_name=desired_function_name)
        # TODO,add bug fix mechanism for such cases,rather than just raising exception to triger retry
        if error:
            self._context.logger.log(f"Function {desired_function_name} has syntax error: {error}", "error")
            raise Exception
        return code


if __name__ == "__main__":
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/test'
    root_manager.update_root(path, set_data_path=False)

    write_run = WriteRun('write run')
    write_run.context.load_from_file(path + "/write_functions.pkl")
    asyncio.run(write_run.run())

    write_run.context.save_to_file(f'{path}/write_run.pkl')
