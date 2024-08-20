from modules.framework.action import ActionNode
from modules.framework.code import FunctionTree, State
from modules.framework.parser import SingleFunctionParser, parse_text
from modules.prompt import (
    WRITE_RUN_PROMPT_TEMPLATE,
    ROBOT_API,
    ENV_DES,
    TASK_DES,
)


class WriteRun(ActionNode):
    def __init__(self, next_text="", node_name=""):
        super().__init__(next_text, node_name)
        self._function_pool = FunctionTree()

    def _build_prompt(self):
        functions = "\n\n".join(self._function_pool.function_valid_content)
        self.prompt = WRITE_RUN_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            env_des=ENV_DES,
            robot_api=ROBOT_API,
            functions=functions,
        )

    async def _process_response(self, response: str) -> str:
        desired_function_name = "run_loop"
        code = parse_text(text=response)
        parser = SingleFunctionParser()
        parser.parse_code(code)
        parser.check_function_name(desired_function_name)

        self._function_pool.update_from_parser(parser.imports, parser.function_dict)
        self._function_pool.save_code([desired_function_name])
        self._function_pool[desired_function_name].state = State.WRITTEN
