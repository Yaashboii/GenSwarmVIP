from modules.framework.action import ActionNode
from modules.framework.response.code_parser import CodeParser
from modules.llm import GPT
from modules.prompt import (
    FEEDBACK_PROMPT_TEMPLATE,
    CONTINUE_FEEDBACK_PROMPT_TEMPLATE,
    ROBOT_API,
    ENV_DES,
    TASK_DES,
)
from modules.framework.response.text_parser import parse_text
from modules.framework.code.function_tree import FunctionTree


class Criticize(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):
        super().__init__(next_text, node_name)
        self.__llm = GPT(memorize=True)
        self.feedback = None
        self._function_pool = FunctionTree()
        self._call_times = 0

    def _build_prompt(self):
        if self._call_times == 0:
            self.prompt = FEEDBACK_PROMPT_TEMPLATE.format(
                task_des=TASK_DES,
                robot_api=ROBOT_API,
                env_des=ENV_DES,
                functions="\n\n\n".join(self._function_pool.functions_body),
                feedback=self.feedback,
            )
        else:
            self.prompt = CONTINUE_FEEDBACK_PROMPT_TEMPLATE.format(
                feedback=self.feedback,
            )
        self._call_times += 1

    def setup(self, feedback: str):
        self.feedback = feedback

    async def _process_response(self, response: str, **kwargs) -> str:
        code = parse_text(text=response)
        parser = CodeParser()
        parser.parse_code(code)
        function_list = parser.function_names
        self._function_pool.update_from_parser(parser.imports, parser.function_dict)
        self._function_pool.save_functions_to_file()
        return str(code)


if __name__ == "__main__":
    critic = Criticize("constraints")
    from modules.utils import root_manager
    import asyncio

    path = "../../../workspace/test"
    root_manager.update_root(path)
    critic.context.load_from_file(path + "/run_code.pkl")
    asyncio.run(critic.run())
    critic.context.save_to_file(f"{path}/analyze_constraints.pkl")
