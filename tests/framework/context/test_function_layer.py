from modules.framework.code import FunctionLayer, FunctionNode


import unittest


class TestFunctionLayer(unittest.TestCase):
    def setUp(self):
        self.function1 = FunctionNode(name="func1", description="Function 1")
        self.function2 = FunctionNode(name="func2", description="Function 2")
        self.function3 = FunctionNode(name="func3", description="Function 3")
        self.function_layer = FunctionLayer({self.function1, self.function2})

    def test_add_function(self):
        self.function_layer.add_function(self.function3)
        self.assertIn(self.function3, self.function_layer._layer)

    def test_functions_property(self):
        self.assertCountEqual(
            self.function_layer.functions, [self.function1, self.function2]
        )

    def test_next_function(self):
        with self.assertRaises(StopIteration):
            for _ in range(len(self.function_layer) + 1):
                next(self.function_layer)

    def test_empty_layer(self):
        function_layer = FunctionLayer()
        self.assertEqual(len(function_layer), 0)
