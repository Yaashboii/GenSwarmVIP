from modules.framework.action import ActionNode
from modules.framework.response.code_parser import SingleFunctionParser
from modules.framework.response.text_parser import parse_text
from modules.prompt.coding_stage_prompt import WRITE_RUN_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.file.log_file import logger
from modules.framework.code.function_tree import FunctionTree

class WriteRun(ActionNode):
    def __init__(self, next_text, node_name = ''):
        super().__init__(next_text, node_name)
        self._function_pool = FunctionTree()

    def _build_prompt(self):
        functions = '\n\n'.join(self._function_pool.function_valid_content)
        self.prompt = WRITE_RUN_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            env_des=ENV_DES,
            robot_api=ROBOT_API,
            functions=functions,
        )

    def _process_response(self, response: str) -> str:
        desired_function_name = "run_loop"
        code = parse_text(text=response)
        parser = SingleFunctionParser()
        parser.parse_code(code)
        parser.check_function_name(desired_function_name)

        self._function_pool.update_from_parser(parser.imports, parser.function_dict)
        self._function_pool.save_code([desired_function_name])
        # self._function_pool.check_grammar([desired_function_name])

        # TODO,add bug fix mechanism for such cases,rather than just raising exception to triger retry
        # if error:
        #     logger.log(f"Function {desired_function_name} has syntax error: {error}", "error")
        #     raise Exception
        # return code

