import os
import unittest
from unittest.mock import patch

from modules.framework.code.grammar_checker import GrammarChecker
from modules.utils.root import root_manager


class TestGrammarChecker(unittest.TestCase):
    def setUp(self):
        self.grammar_checker = GrammarChecker()
        self.file_path = "tests/framework/code/example.py"

        self.errors = [
            {
                "file_path": self.file_path,
                "line": 3,
                "column": 11,
                "error_code": "E0602",
                "error_message": "Undefined variable 'd'",
            },
            {
                "file_path": self.file_path,
                "line": 8,
                "column": 27,
                "error_code": "E0602",
                "error_message": "Undefined variable 'global_variable'",
            },
            {
                "file_path": self.file_path,
                "line": 14,
                "column": 14,
                "error_code": "E1101",
                "error_message": "Instance of 'BadClass' has no 'mesage' member; maybe 'message'?",
            },
        ]

    @patch("modules.file.file.logger.log")
    def test_run_pylint_check(self, mock_logger):
        errors = self.grammar_checker._run_pylint_check(self.file_path)
        print(errors)
        self.assertEqual(errors, self.errors)

    def test_find_function_name_from_error(self):
        (
            function_name,
            error_code_line,
        ) = self.grammar_checker._find_function_name_from_error(
            os.path.join(root_manager.project_root, self.file_path), 3
        )

        self.assertEqual(function_name, "my_function")
        self.assertEqual(error_code_line, "return d")

    # @patch("modules.file.file.logger.log")
    # def test_check_code_errors(self, mock_logger):
    #     errors = self.grammar_checker.check_code_errors(
    #         os.path.join(root_manager.project_root, self.file_path)
    #     )
    #     self.assertEqual(errors, self.errors)


if __name__ == "__main__":
    unittest.main()
