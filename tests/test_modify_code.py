import unittest
import logging

from modules.framework.actor import Actor
from modules.framework.critic import CodeErrorCritic

class TestCode(unittest.TestCase):
    def setUp(self) -> None:
        self._actor = Actor()
        self._code_critic = CodeErrorCritic()

    def test_modify_code(self):
        # query = ""
        # code = self._actor.generate_code(query, log=False)
        code = """
c = a + b
print("the result is :", c) 
        """
        print("Origin code: ", code)
        is_passed, suggection = self._code_critic.evaluate(code)
        print("first try: ", is_passed, "\nSuggestion: ", suggection)
        if not is_passed:
            code = self._actor.modify_code(code, suggection)
            print("Regenerated code: ", code)

        is_passed, suggection = self._code_critic.evaluate(code)
        self.assertTrue(is_passed, "Final modified code should pass evaluation.")



if __name__ == '__main__':
    logging.basicConfig(level=logging.CRITICAL)
    # 创建测试套件
    suite = unittest.TestSuite()
    # 将特定的测试方法添加到测试套件
    suite.addTest(TestCode('test'))

    # 运行测试套件
    runner = unittest.TextTestRunner()
    runner.run(suite)
    # unittest.main()