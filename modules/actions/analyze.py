import json

from modules.actions.action import Action
from modules.utils import parse_code


class AnalyzeConstraints(Action):
    name: str = "AnalyzeConstraints"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response, lang='json')
        self._context.constraint_pool.add_constraints(code)
        self._context.log.format_message(f"Analyze Constraints Success", "success")
        return response


class AnalyzeFunctions(Action):
    name: str = "AnalyzeFunctions"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response, lang='json')
        self._context.function_pool.init_functions(code)
        for function in self._context.function_pool.functions.values():
            for constraint in function.satisfying_constraints:
                self._context.constraint_pool.add_sat_func(constraint_name=constraint, function_name=function.name)
        for constraint in self._context.constraint_pool.constraints.values():
            if not constraint.satisfyingFuncs:
                raise SystemExit(f"Constraint {constraint.name} has no satisfying function")
        self._context.log.format_message(f"Analyze Functions Success", "success")
        return response
