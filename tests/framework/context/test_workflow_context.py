import unittest
import os

from modules.framework.code import FunctionTree
from modules.framework.context import WorkflowContext
from modules.framework.constraint import ConstraintPool


class TestWorkflowContext(unittest.TestCase):
    TEST_FILE_PATH = os.path.dirname(os.path.abspath(__file__)) + "/test_context.pkl"

    @classmethod
    def setUpClass(cls):
        cls.context = WorkflowContext()
        cls.constraint_pool = cls.context._constraint_pool
        cls.function_pool = cls.context._function_pool

    @classmethod
    def tearDownClass(cls):
        if os.path.exists(cls.TEST_FILE_PATH):
            os.remove(cls.TEST_FILE_PATH)

    def test_save_and_load_constraint_pool(self):
        expected_dict = {"constraint1": "value1", "constraint2": "value2"}
        self.constraint_pool._constraint_nodes = expected_dict
        self.context.save_to_file(self.TEST_FILE_PATH)

        self.constraint_pool.reset()
        constraint_data = ConstraintPool()._constraint_nodes
        self.assertEqual(len(constraint_data), 0)

        self.context.load_from_file(self.TEST_FILE_PATH)
        constraint_data = ConstraintPool()._constraint_nodes
        self.assertEqual(constraint_data["constraint1"], "value1")
        self.assertEqual(constraint_data["constraint2"], "value2")

    def test_save_and_load_function_pool(self):
        import_data = {"from apis import *", "import 1", "import 2"}
        self.function_pool.import_list = import_data
        self.context.save_to_file(self.TEST_FILE_PATH)
        self.function_pool.reset()

        function_pool_imports = FunctionTree().import_list
        self.assertEqual(len(function_pool_imports), 1)

        self.context.load_from_file(self.TEST_FILE_PATH)

        function_pool_imports = FunctionTree().import_list
        self.assertCountEqual(function_pool_imports, import_data)


if __name__ == "__main__":
    unittest.main()
