import asyncio
from modules.framework.code_error import *
from modules.framework.action import *
from modules.framework.handler import *


class WriteCode(ActionNode):
    pass

class WriteTest(ActionNode):
    pass

class CriticCheck(ActionNode):
    def _process_response(self, response: str) -> str:
        return CriticNotSatisfied()
        pass

write_code = WriteCode("code")
write_test = WriteTest("test result")

bug_handler = BugLevelHandler()
critical_handler = CriticLevelHandler()
hf_handler = HumanFeedbackHandler()
handler_chain = bug_handler
bug_handler.successor = critical_handler
critical_handler.successor = hf_handler

b = ActionLinkedList("code and test", write_code)
b.add(write_test)
c1 = CriticCheck("C1 result")
c1.error_handler = handler_chain
c2 = CriticCheck("C2 result", node_name="c2check")
c2.error_handler = handler_chain
d = ActionLinkedList("Criteria Check", c1)
d.add(c2)
main = ActionLinkedList("main", b)
main.add(d)
final = ActionNode(next_text="n", node_name="Final")
main.add(final)


critical_handler.next_action = d



text = display_all(main, handler_chain)

import os
current_dir = os.path.dirname(os.path.abspath(__file__))
filepath = os.path.join(current_dir, 'flow.md')

# 写入文本到文件
with open(filepath, 'w') as file:
    file.write(text)
