import ast

class Code:
    def __init__(self, code_str) -> None:
        self._code_str = code_str
        self._imports = []
        self._functions = []

    def extract_imports_and_functions(self):
        parsed_ast = ast.parse(self._code_str)
        for node in parsed_ast.body:
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        import_str = f"import {alias.name}"
                        if alias.asname:
                            import_str += f" as {alias.asname}"
                        self._imports.append(import_str)
                elif isinstance(node, ast.ImportFrom):
                    module = node.module if node.module else ''
                    import_from_str = "from {} import ".format(module)
                    names_with_as = []
                    for alias in node.names:
                        if alias.asname:
                            names_with_as.append(f"{alias.name} as {alias.asname}")
                        else:
                            names_with_as.append(alias.name)
                    import_from_str += ", ".join(names_with_as)
                    self._imports.append(import_from_str)
            elif isinstance(node, ast.FunctionDef):
                func_def = ast.unparse(node).strip()
                if func_def:
                    self._functions.append(func_def)

        return self._imports, self._functions

    def extract_top_level_function_names(self) -> list[str]:
        tree = ast.parse(self._code_str)

        def add_parent_references(node, parent=None):
            node.parent = parent
            for child in ast.iter_child_nodes(node):
                add_parent_references(child, node)

        add_parent_references(tree)

        function_names = []

        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and isinstance(node.parent, ast.Module):
                function_names.append(node.name)

        return function_names