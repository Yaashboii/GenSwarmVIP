import asyncio
from modules.framework.code_error import *
from modules.framework.action import *
from modules.framework.handler import *

class UserInput(ActionNode):
    pass

class WriteAnalyse(ActionNode):
    pass

class WriteDesign(ActionNode):
    pass

class WriteCode(ActionNode):
    pass

class WriteTest(ActionNode):
    pass

class CriticCheck(ActionNode):
    def _process_response(self, response: str) -> str:
        return CriticNotSatisfied()

user_input = UserInput("command")
write_analysis = WriteAnalyse("analysis")
write_design = WriteDesign("API design")
write_code = WriteCode("code")
write_test = WriteTest("test result")

a = ActionLinkedList("analysis and design", write_analysis)
a.add(write_design)
b = ActionLinkedList("code and test", write_code)
b.add(write_test)
main = ActionLinkedList("total", user_input)
main.add(a)
main.add(b)
c1 = CriticCheck("C1 result")
c2 = CriticCheck("C2 result", "c2check")
d = ActionLinkedList("Criteria Check", c1)
d.add(c2)
main.add(d)
text = display_all(main)

import os
current_dir = os.path.dirname(os.path.abspath(__file__))
filepath = os.path.join(current_dir, 'flow.md')

# 写入文本到文件
with open(filepath, 'w') as file:
    file.write(text)
