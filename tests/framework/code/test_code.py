import unittest

from modules.framework.code.parser import _AstParser, parse_text

class TestAstParser(unittest.TestCase):

    def setUp(self):
        self.code_str = """
import math
from collections import deque as d

def add(a, b=0):
    \"\"\"This function adds two numbers.\"\"\"
    return a + b

def subtract(x, y=0):
    \"\"\"This function subtracts two numbers.\"\"\"
    return x - y
        """

        self.parser = _AstParser(self.code_str)

    def test_imports(self):
        expected_imports = {"import math", "from collections import deque as d"}
        self.assertEqual(self.parser.imports, expected_imports)

    def test_function_names(self):
        expected_function_names = {"add", "subtract"}
        self.assertEqual(set(self.parser.function_names), expected_function_names)

    def test_function_contents(self):
        expected_function_contents = [
            'def add(a, b=0):\n    """This function adds two numbers."""\n    return a + b',
            'def subtract(x, y=0):\n    """This function subtracts two numbers."""\n    return x - y'
        ]
        self.assertEqual(list(self.parser.function_contents), expected_function_contents)

    def test_function_defs(self):
        expected_function_defs = [
            'def add(a, b=0):\n    """\n    This function adds two numbers.\n    """\n',
            'def subtract(x, y=0):\n    """\n    This function subtracts two numbers.\n    """\n'
        ]
        self.assertEqual(self.parser.function_defs, expected_function_defs)

    def test_parse_code(self):
        text = "```python\nprint('Hello, World!')\n```"
        expected_code = "print('Hello, World!')\n"
        parsed_code = parse_text(text)
        self.assertEqual(parsed_code, expected_code)

if __name__ == '__main__':
    unittest.main()
