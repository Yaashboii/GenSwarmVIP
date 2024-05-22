import os
import unittest
from unittest.mock import patch, MagicMock, mock_open

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

    @patch("modules.file.file.logger.log")
    @patch.object(GrammarChecker, "_run_pylint_check")
    @patch.object(GrammarChecker, "_find_function_name_from_error")
    def test_check_code_errors_with_errors(
        self, mock_find_function_name, mock_run_pylint_check, mock_logger
    ):
        # 设置模拟数据
        mock_run_pylint_check.return_value = [
            {"line": 5, "error_message": "Syntax error"}
        ]
        mock_find_function_name.return_value = ("my_function", "return d")

        # 运行被测试方法
        result = self.grammar_checker.check_code_errors("test_file.py")

        # 断言方法返回了错误列表
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["line"], 5)
        self.assertEqual(result[0]["error_message"], "Syntax error")

        # 断言日志被正确记录
        mock_logger.assert_called_with("my_function: Syntax error", level="error")

        # 断言每个错误都包含了函数名
        self.assertEqual(result[0]["function_name"], "my_function")

    @patch("modules.file.file.logger.log")
    @patch.object(GrammarChecker, "_run_pylint_check")
    @patch.object(GrammarChecker, "_find_function_name_from_error")
    def test_check_code_errors_without_errors(
        self, mock_find_function_name, mock_run_pylint_check, mock_logger
    ):
        # 设置模拟数据，表示没有错误
        mock_run_pylint_check.return_value = []

        # 运行被测试方法
        result = self.grammar_checker.check_code_errors("test_file.py")

        # 断言方法返回了空列表
        self.assertEqual(result, [])

        # 断言日志未被调用，因为没有错误
        mock_logger.assert_not_called()

    @patch("modules.file.file.logger.log")
    def test_run_pylint_check_exception_handling(self, mock_logger):
        # 设置模拟数据，使模拟的方法引发异常
        with patch("subprocess.run") as mock_run:
            mock_run.side_effect = Exception("Mocked exception")

            # 运行被测试方法
            with self.assertRaises(Exception) as context:
                self.grammar_checker._run_pylint_check("test_file.py")

            # 断言异常被正确地抛出
            self.assertEqual(
                str(context.exception),
                "Error occurred when check grammar: Mocked exception",
            )

            # 断言异常时日志被正确记录
            mock_logger.assert_called_with(
                "Error occurred when check grammar: Mocked exception", level="error"
            )

    @patch(
        "builtins.open",
        new_callable=mock_open,
        read_data="def my_function():\n    return 42\n",
    )
    def test_find_function_name_from_error_no_function(self, mock_open):
        # 运行被测试方法
        (
            function_name,
            error_code_line,
        ) = self.grammar_checker._find_function_name_from_error("fake_file.py", 1)

        # 断言没有找到函数名，但找到了错误行
        self.assertIsNone(function_name)
        self.assertEqual(error_code_line, "def my_function():")


if __name__ == "__main__":
    unittest.main()
