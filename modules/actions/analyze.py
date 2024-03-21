import json

from modules.actions.action import Action
from modules.utils import parse_code


class AnalyzeConstraints(Action):
    name: str = "AnalyzeConstraints"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response, lang='json')
        self._context.constraints.add_constraints(code)
        self._context.log.format_message(f"Analyze Constraints Success", "success")
        return response


class AnalyzeFunctions(Action):
    name: str = "AnalyzeFunctions"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response, lang='json')
        self._context.functions.init_functions(code)
        for i, function in self._context.functions.functions.items():
            for constraint in function.satisfying_constraints:
                self._context.constraints.add_sat_func(constraint_name=constraint, function_id=i)
            for constraint in self._context.constraints.constraints.values():
                if not constraint.satisfyingFuncs:
                    print(f"Constraint {constraint.name} has no satisfying function")
                    raise SystemExit
        self._context.log.format_message(f"Analyze Functions Success", "success")
        return response
