from modules.framework.action import ActionNode
from modules.prompt.run_code_prompt import HUMAN_FEEDBACK_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code, extract_top_level_function_names


class GrammarFeedback(ActionNode):
    def __init__(self, next_text: str = '', node_name: str = ''):
        super().__init__(next_text, node_name)
        self.feedback = None

    def _build_prompt(self):
        self.prompt = HUMAN_FEEDBACK_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            functions=self.context.function_content,
            feedback=self.feedback,
        )

    def setup(self, feedback: str):
        self.feedback = feedback

    def _process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response)
        function_list = extract_top_level_function_names(code)
        self.context.add_functions(content=code)

        for function_name in function_list:
            self.context.check_function_grammar(function_name=function_name)
            for function in self.context.functions_value:
                if function_name in function.calls:
                    self.context.check_function_grammar(function_name=function.name)
        return str(code)

if __name__ == '__main__':
    critic = GrammarFeedback("constraints")
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/test'
    root_manager.update_root(path, set_data_path=False)
    critic.context.load_from_file(path + "/run_code.pkl")
    asyncio.run(critic.run())
    critic.context.save_to_file(f'{path}/analyze_constraints.pkl')
