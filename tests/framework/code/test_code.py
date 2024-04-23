import unittest

from modules.framework.code.code import Code 

class TestCode(unittest.TestCase):
    
    def setUp(self):
        self.code_str = """
import math
from os import path

def add(a, b):
    return a + b
def subtract(a, b):
    return a - b
        """
        self.code = Code(self.code_str)

    def test_extract_imports_and_functions(self):
        expected_imports = ['import math', 'from os import path']
        expected_functions = ['def add(a, b):\n    return a + b', 'def subtract(a, b):\n    return a - b']
        
        imports, functions = self.code.extract_imports_and_functions()
        
        self.assertEqual(imports, expected_imports)
        self.assertEqual(functions, expected_functions)

    def test_extract_top_level_function_names(self):
        expected_function_names = ['add', 'subtract']
        
        function_names = self.code.extract_top_level_function_names()
        
        self.assertEqual(function_names, expected_function_names)

    def test_extract_function_definitions(self):
        expected_definitions = [
            "def add(a, b):\n",
            "def subtract(a, b):\n"
        ]
        
        definitions = self.code.extract_function_definitions()
        
        self.assertEqual(definitions, expected_definitions)

if __name__ == '__main__':
    unittest.main()
