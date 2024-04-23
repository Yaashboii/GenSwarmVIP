from modules.framework.action import ActionNode
from modules.framework.code.code import AstParser
from modules.prompt.run_code_prompt import HUMAN_FEEDBACK_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.framework.code.code import parse_code
from modules.framework.context import FunctionPool

class GrammarFeedback(ActionNode):
    def __init__(self, next_text: str = '', node_name: str = ''):
        super().__init__(next_text, node_name)
        self.feedback = None
        self._function_pool = FunctionPool()

    def _build_prompt(self):
        self.prompt = HUMAN_FEEDBACK_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            functions=self._function_pool.functions_content(),
            feedback=self.feedback,
        )

    def setup(self, feedback: str):
        self.feedback = feedback

    def _process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        code_obj = AstParser(code)
        function_list = code_obj.extract_top_level_function_names()
        self._function_pool.add_functions(content=code)

        for function_name in function_list:
            self._function_pool.check_function_grammar(function_name)
            self._function_pool.check_caller_function_grammer(function_name)
        return str(code)

if __name__ == '__main__':
    critic = GrammarFeedback("constraints")
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/test'
    root_manager.update_root(path)
    critic.context.load_from_file(path + "/run_code.pkl")
    asyncio.run(critic.run())
    critic.context.save_to_file(f'{path}/analyze_constraints.pkl')
