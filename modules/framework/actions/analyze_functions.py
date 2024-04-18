from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import ANALYZE_FUNCTION_PROMPT_TEMPLATE, FUNCTION_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code
from modules.framework.context import logger, ConstraintPool

class AnalyzeFunctions(ActionNode):
    def __init__(self, next_text, node_name = ''):
        super().__init__(next_text, node_name)
        self._constraint_pool : ConstraintPool = ConstraintPool()

    def _build_prompt(self):
        self.prompt = ANALYZE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self.context.command,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            constraints=str(self._constraint_pool),
            output_template=FUNCTION_TEMPLATE
        )

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response, lang='json')
        self.context.init_functions(code)

        for function in self.context.functions_value(code):
            for constraint in function.satisfying_constraints:
                self._constraint_pool.add_satisfying_func(constraint_name=constraint, function_name=function.name)

        self._constraint_pool.check_invalid_constraints()

        logger.log(f"Analyze Functions Success", "success")
        return response

if __name__ == '__main__':
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/test'
    root_manager.update_root(path)

    analyze_functions = AnalyzeFunctions('functions')
    analyze_functions.context.load_from_file(path + "/analyze_constraints.pkl")
    asyncio.run(analyze_functions.run())

    analyze_functions.context.save_to_file(f'{path}/analyze_functions.pkl')
