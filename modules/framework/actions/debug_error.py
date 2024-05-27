from modules.framework.action import ActionNode
from modules.framework.response.text_parser import parse_text
from modules.prompt.run_code_prompt import DEBUG_PROMPT, CONTINUE_DEBUG_PROMPT
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.task_description import TASK_DES
from modules.framework.code.function_tree import FunctionTree
from modules.framework.response.code_parser import CodeParser
from modules.llm.gpt import GPT


class DebugError(ActionNode):
    def __init__(self, next_text="", node_name=""):
        super().__init__(next_text, node_name)
        self.__llm = GPT(memorize=True)
        self.error = None
        self._function_pool = FunctionTree()
        self._call_times = 0

    def setup(self, error):
        self.error = error

    def _build_prompt(self):
        if self._call_times == 0:
            mentioned_function = "\n\n".join(
                self._function_pool.related_function_content(self.error)
            )
            self.prompt = DEBUG_PROMPT.format(
                task_des=TASK_DES,
                robot_api=ROBOT_API,
                env_des=ENV_DES,
                mentioned_functions=mentioned_function,
                error_message=self.error,
            )
        else:
            self.prompt = CONTINUE_DEBUG_PROMPT.format(
                error_message=self.error,
            )
        self._call_times += 1

    async def _process_response(self, response: str, **kwargs) -> str:
        code = parse_text(text=response)
        parser = CodeParser()
        parser.parse_code(code)
        self._function_pool.update_from_parser(parser.imports, parser.function_dict)
        self._function_pool.save_functions_to_file()

        return str(code)
