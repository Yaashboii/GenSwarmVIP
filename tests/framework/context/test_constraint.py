import unittest
from unittest.mock import patch, mock_open
from modules.framework.context.contraint_info import ConstraintPool, ConstraintNode

class TestConstraintPool(unittest.TestCase):

    def setUp(self):
        self.constraint_pool : ConstraintPool = ConstraintPool()

    def test_singleton_instance(self):
        # 测试 ConstraintPool 是否为单例模式
        another_instance = ConstraintPool()
        self.assertIs(self.constraint_pool, another_instance)

    def test_init_constraints(self):
        # 测试 init_constraints 方法
        content = '{"constraints": [{"name": "test1", "description": "Test Constraint 1"}, {"name": "test2", "description": "Test Constraint 2"}]}'
        self.constraint_pool.init_constraints(content)
        
        self.assertIn('test1', self.constraint_pool._constraint_nodes)
        self.assertIn('test2', self.constraint_pool._constraint_nodes)

        print("root: ", self.constraint_pool._file._root)
    
    
    # @patch('builtins.open', mock_open(read_data='existing content'))
    # def test_init_constraints_sync_to_file(self):
    #     # 测试 init_constraints 方法和 sync_to_file 方法
    #     content = '{"constraints": [{"name": "test1", "description": "Test Constraint 1"}]}'
    #     self.constraint_pool.init_constraints(content)
        
    #     # 检查 _file 属性是否更新了
    #     expected_content = 'existing content\n{"constraints": [{"name": "test1", "description": "Test Constraint 1"}]}'
    #     self.assertEqual(self.constraint_pool._file.message, expected_content)

    # def test_getitem(self):
    #     # 测试 __getitem__ 方法
    #     constraint_node = ConstraintNode(name="test_getitem", description="Test GetItem")
    #     self.constraint_pool._constraint_nodes["test_getitem"] = constraint_node
        
    #     retrieved_constraint = self.constraint_pool["test_getitem"]
    #     self.assertIs(retrieved_constraint, constraint_node)
        
    #     with self.assertRaises(SystemExit):
    #         _ = self.constraint_pool["non_existing_constraint"]

    # def test_constaint_list(self):
    #     # 测试 constaint_list 属性
    #     constraint_node = ConstraintNode(name="test_list", description="Test List")
    #     self.constraint_pool._constraint_nodes["test_list"] = constraint_node
        
    #     constraint_list = self.constraint_pool.constaint_list
    #     self.assertEqual(len(constraint_list), 1)
    #     self.assertEqual(constraint_list[0]["name"], "test_list")
        
    # def test_filtered_constaints(self):
    #     # 测试 filtered_constaints 方法
    #     constraint_node1 = ConstraintNode(name="test1", description="Test 1")
    #     constraint_node2 = ConstraintNode(name="test2", description="Test 2")
        
    #     self.constraint_pool._constraint_nodes["test1"] = constraint_node1
    #     self.constraint_pool._constraint_nodes["test2"] = constraint_node2
        
    #     filtered_constraints = self.constraint_pool.filtered_constaints(["test1"])
    #     self.assertIn("Test 1", filtered_constraints)
    #     self.assertNotIn("Test 2", filtered_constraints)

    # def test_check_constraints_satisfaction(self):
    #     # 测试 check_constraints_satisfaction 方法
    #     constraint_node1 = ConstraintNode(name="test1", description="Test 1")
    #     constraint_node2 = ConstraintNode(name="test2", description="Test 2")
        
    #     self.constraint_pool._constraint_nodes["test1"] = constraint_node1
    #     self.constraint_pool._constraint_nodes["test2"] = constraint_node2
        
    #     with self.assertRaises(SystemExit):
    #         self.constraint_pool.check_constraints_satisfaction()

if __name__ == '__main__':
    unittest.main()