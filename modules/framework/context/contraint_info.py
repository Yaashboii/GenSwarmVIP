from modules.file.file import File, logger
from modules.framework.context.node import ConstraintNode
from modules.framework.context import ConstraintNode

class ConstraintPool():
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._constraint_nodes: dict[str, ConstraintNode] = {}
            cls._file = File("constraints.md")
        return cls._instance
    
    def __str__(self) -> str:
        result = '\n'.join([c.brief for c in self._constraint_nodes.values()])
        return result
    
    def __getitem__(self, constraint_name):
        if constraint_name in self._constraint_nodes:
            return self._constraint_nodes[constraint_name]
        else:
            logger.log(
                (f"Constraint {constraint_name} not found,Current Existing:")
                ("{self._constraint_nodes.values()}",level='error')
            )
            raise SystemExit
    
    @property
    def constaint_list(self):
        result = [constraint.to_json() for constraint in self._constraint_nodes.values()]
        return result
    
    def filtered_constaints(self, keys):
        def check(constraint):
            if constraint not in self._constraint_nodes:
                logger.log(f"Constraint {constraint} is not in the constraint pool", 'error')
                raise SystemExit
            
        [check(key) for key in keys]
        result = '\n'.join([c.brief for c in self._constraint_nodes.values() if c in keys])
        return result
    
    def init_constraints(self, content: str):
        def sync_to_file():
            self._file.message = str(self)

        try:
            for constraint in eval(content)['constraints']:
                name = constraint['name']
                self._constraint_nodes[name] = ConstraintNode(
                    name=name,
                    description=constraint['description']
                )
        except Exception as e:
            logger.log(f'Error in init_constraints: {e}', level='error')
            raise Exception
        sync_to_file()
             
        
    def check_constraints_satisfaction(self):
        def report_error(constraint: ConstraintNode):
            raise SystemExit(f"Constraint {constraint._name} has no satisfying function")
        
        [report_error(c) for c in self._constraint_nodes.values() if c.has_no_connections()]


__all__ = ['ConstraintPool']