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


    def extract_function_definitions(self):
        parsed_ast = ast.parse(self._code_str)

        def reconstruct_function_definition(function_node):
            defaults_start_index = len(function_node.args.args) - len(function_node.args.defaults)

            parameters = [
                ast.unparse(arg) + (
                    f'={ast.unparse(function_node.args.defaults[i - defaults_start_index])}' if i >= defaults_start_index else '')
                for i, arg in enumerate(function_node.args.args)
            ]

            func_header = f"def {function_node.name}({', '.join(parameters)}):"
            docstring = ast.get_docstring(function_node)
            docstring_part = ''
            if docstring:
                indented_docstring = '\n'.join('    ' + line for line in docstring.split('\n'))
                docstring_part = f'    """\n{indented_docstring}\n    """\n'
            body_part = ''
            return f"{func_header}\n{docstring_part}{body_part}"

        function_definitions = [reconstruct_function_definition(node) for node in ast.walk(parsed_ast) if
                                isinstance(node, ast.FunctionDef)]
        

        return function_definitions


