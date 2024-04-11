from modules.framework.context.file_info import FileInfo, logger


class ConstraintInfo:
    def __init__(self, description, name):
        self.satisfyingFuncs: list[int] = []
        self.name = name
        self.description = description
        self.text = f"**{self.name}**: {self.description}"

class ConstraintPool(FileInfo):
    def __init__(self, name: str = '', root: str = ''):
        super().__init__(name=name, root=root)
        self.constraints: dict[str, ConstraintInfo] = {}

    def add_constraints(self, content: str):
        try:
            for constraint in eval(content)['constraints']:
                name = constraint['name']
                self.constraints[name] = ConstraintInfo(
                    name=name,
                    description=constraint['description']
                )
        except Exception as e:
            logger.log(f'Error in add_constraints: {e}', level='error')
            raise Exception
        self.update_message()

    def update_message(self):
        self.message = self.constraints_content()

    def add_sat_func(self, constraint_name, function_name: str | list[str]):
        if isinstance(function_name, str):
            function_name = [function_name]
        if constraint_name in self.constraints:
            self.constraints[constraint_name].satisfyingFuncs.extend(function_name)
        else:
            logger.log(f"Constraint {constraint_name} not found,Current Existing:{self.constraints.values()}",
                       level='error')
            raise SystemExit

    def constraints_content(self):
        return '\n'.join([c.text for c in self.constraints.values()])
