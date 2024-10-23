from modules.framework.action import ActionNode
from modules.framework.code import FunctionTree
from modules.framework.code_error import CodeError, Bug, Bugs
from modules.framework.parser import parse_text, CodeParser
from modules.llm import GPT
from modules.prompt import (
    DEBUG_PROMPT,
    CONTINUE_DEBUG_PROMPT,
    ALLOCATOR_TEMPLATE,
    GLOBAL_ROBOT_API,
    LOCAL_ROBOT_API,
    ENV_DES,
    TASK_DES,
)


class DebugError(ActionNode):
    def __init__(self, next_text="", node_name=""):
        super().__init__(next_text, node_name)
        self.__llm = GPT(memorize=True)
        self.error = None
        self.error_func = None
        self._skill_tree = None

    def setup(self, error: CodeError | Bugs | Bug):
        self.error = error.error_msg
        self.error_func = error.error_code
        self._skill_tree = self.context.local_skill_tree if self.context.scoop == "local" else self.context.global_skill_tree

    def _build_prompt(self):
        if len(self.context.global_skill_tree.layers) == 0:
            local_api_prompt = LOCAL_ROBOT_API
        else:
            local_api_prompt = LOCAL_ROBOT_API + ALLOCATOR_TEMPLATE.format(
                template=self.context.global_skill_tree.output_template)
        robot_api = GLOBAL_ROBOT_API if self.context.scoop == "global" else local_api_prompt
        # if self._call_times == 0:
        self.prompt = DEBUG_PROMPT.format(
            task_des=TASK_DES,
            robot_api=robot_api,
            env_des=ENV_DES,
            mentioned_functions=self.error_func,
            error_message=self.error,
        )
        # else:
        #     self.prompt = CONTINUE_DEBUG_PROMPT.format(
        #         error_message=self.error,
        #     )
        # self._call_times += 1

    async def _process_response(self, response: str, **kwargs) -> str:
        code = parse_text(text=response)
        parser = CodeParser()
        parser.parse_code(code)
        self._skill_tree.update_from_parser(parser.imports, parser.function_dict)
        self._skill_tree.save_functions_to_file()

        return str(code)
