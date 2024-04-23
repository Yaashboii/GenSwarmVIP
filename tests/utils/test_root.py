import unittest
import os
from modules.utils.root import root_manager
from pathlib import Path


class TestRootManager(unittest.TestCase):

    def setUp(self):
        self.root_manager = root_manager

    def test_get_project_root(self):
        #TODO: get path by code
        supposed_dir = Path('/src')
        actual_dir = self.root_manager.get_project_root()
        self.assertEqual(actual_dir, supposed_dir, f"Expected {supposed_dir}, got {actual_dir}")
        
    def test_update_root_with_workspace_root(self):
        # Mock the provided workspace root
        fake_workspace_root = Path("/fake_workspace_root")
        self.root_manager.update_root(workspace_root=fake_workspace_root)
        self.assertEqual(self.root_manager.workspace_root, fake_workspace_root)
        self.assertEqual(self.root_manager.data_root, fake_workspace_root / "data")

if __name__ == '__main__':
    unittest.main()
