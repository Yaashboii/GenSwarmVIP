from modules.framework.context.node import ConstraintNode, FunctionNode

class Linker:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            super().__new__(cls)
            cls._contraint_nodes = set()
            cls._function_nodes = set()
        return cls._instance
    
    
    
