from modules.framework.action import ActionNode
from modules.prompt.analyze_stage_prompt import ANALYZE_FUNCTION_PROMPT_TEMPLATE, FUNCTION_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.utils import parse_code


class AnalyzeFunctions(ActionNode):
    def _build_prompt(self):
        self.prompt = ANALYZE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            instruction=self._context.user_command.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            constraints=self._context.constraint_pool.message,
            output_template=FUNCTION_TEMPLATE
        )

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response, lang='json')
        self._context.function_pool.init_functions(code)

        for function in self._context.function_pool.functions.values():
            for constraint in function.satisfying_constraints:
                self._context.constraint_pool.add_sat_func(constraint_name=constraint, function_name=function.name)

        for constraint in self._context.constraint_pool.constraints.values():
            if not constraint.satisfyingFuncs:
                raise SystemExit(f"Constraint {constraint.name} has no satisfying function")

        self._context.logger.log(f"Analyze Functions Success", "success")
        return response

    def _can_skip(self) -> bool:
        return False


if __name__ == '__main__':
    from modules.utils import root_manager
    import asyncio

    path = '../../../workspace/test'
    root_manager.update_root(path, set_data_path=False)

    analyze_functions = AnalyzeFunctions('functions')
    analyze_functions.context.load_from_file(path + "/analyze_constraints.pkl")
    asyncio.run(analyze_functions.run())

    analyze_functions.context.save_to_file(f'{path}/analyze_functions.pkl')
