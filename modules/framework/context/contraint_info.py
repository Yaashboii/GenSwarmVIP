from modules.file.file import File, logger


class _ConstraintInfo:
    def __init__(self, description, name):
        self._satisfyingFuncs: list[str] = []
        self._name = name
        self._description = description

    @property
    def constraint_info(self):
        return f"**{self._name}**: {self._description}"
    
    def to_json(self):
        result = {
            "name": self._name,
            "description": self._description
        }
        return result
    
    def add_function_list(self, function_list):
        self._satisfyingFuncs.extend(function_list)

    def is_function_empty(self):
        return len(self._satisfyingFuncs) == 0


class ConstraintPool():
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._constraints: dict[str, _ConstraintInfo] = {}
            cls._file = File("constraints.md")
        return cls._instance
    
    def __str__(self) -> str:
        result = '\n'.join([c.constraint_info for c in self._constraints.values()])
        return result
    
    @property
    def constaint_list(self):
        result = [constraint.to_json() for constraint in self._constraints.values()]
        return result
    
    def filtered_constaints(self, keys):
        def check(constraint):
            if constraint not in self._constraints:
                logger.log(f"Constraint {constraint} is not in the constraint pool", 'error')
                raise SystemExit
            
        [check(key) for key in keys]
        result = '\n'.join([c.constraint_info for c in self._constraints.values() if c in keys])
        return result
    
    def add_constraints(self, content: str):
        try:
            for constraint in eval(content)['constraints']:
                name = constraint['name']
                self._constraints[name] = _ConstraintInfo(
                    name=name,
                    description=constraint['description']
                )
        except Exception as e:
            logger.log(f'Error in add_constraints: {e}', level='error')
            raise Exception
        self._sync_to_file()
        
    def _sync_to_file(self):
        self._file.message = str(self)

    def add_satisfying_func(self, constraint_name, function_name: str | list[str]):
        if isinstance(function_name, str):
            function_list = [function_name]
        elif isinstance(function_name, list):
            function_list = function_name
        if constraint_name in self._constraints:
            self._constraints[constraint_name].add_function_list(function_list)
        else:
            logger.log(f"Constraint {constraint_name} not found,Current Existing:{self._constraints.values()}",
                       level='error')
            raise SystemExit
        
    def check_invalid_constraints(self):
        def report_error(constraint: _ConstraintInfo):
            raise SystemExit(f"Constraint {constraint._name} has no satisfying function")
        [report_error(c) for c in self._constraints.values() if c.is_function_empty()]


__all__ = ['ConstraintPool']