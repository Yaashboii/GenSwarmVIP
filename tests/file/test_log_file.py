import unittest
from unittest.mock import MagicMock
from modules.file.log_file import _Logger
from modules.file.base_file import BaseFile


class TestLogger(unittest.TestCase):
    def setUp(self):
        self.logger = _Logger()
        self.mock_file = MagicMock(spec=BaseFile)

    def test_set_file(self):
        self.logger.set_file(self.mock_file)
        self.assertEqual(self.logger._file, self.mock_file)

    def test_log(self):
        test_content = "Test Content"
        self.logger.set_file(self.mock_file)

        # Test logging at different levels
        levels = [
            "stage",
            "action",
            "prompt",
            "response",
            "success",
            "error",
            "warning",
            "info",
            "debug",
        ]
        for level in levels:
            with self.subTest(level=level):
                self.logger.log(test_content, level)


if __name__ == "__main__":
    unittest.main()
