
from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import ANALYZE_FUNCTION_PROMPT_TEMPLATE, FUNCTION_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.framework.context.contraint_info import ConstraintPool
from modules.framework.code.function_tree import FunctionTree
from modules.file.log_file import logger
from modules.framework.response import *


class AnalyzeFunctions(ActionNode):
    def __init__(self, next_text, node_name = ''):
        super().__init__(next_text, node_name)
        self._constraint_pool : ConstraintPool = ConstraintPool()
        self._function_pool = FunctionTree()

    def _build_parser(self):
        self._parser = TextParser('json')

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
        code = super()._process_response(response)
        self._function_pool.init_functions(code)
        self._constraint_pool.check_constraints_satisfaction()
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
