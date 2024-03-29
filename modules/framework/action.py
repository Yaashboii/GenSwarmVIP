from abc import ABC, abstractmethod

from modules.framework.workflow_context import WorkflowContext
from modules.utils import setup_logger, LoggerLevel
from modules.llm.gpt import GPT
from tenacity import retry, stop_after_attempt, wait_random_exponential

class BaseNode(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)
        self.__next = None

    def __str__(self):
        return self.__class__.__name__

    @property
    def _next(self):
        return self.__next
    
    @_next.setter
    def _next(self, value):
        if not isinstance(value, BaseNode):
            raise ValueError("Value must be a BaseNode")
        self.__next = value

    @abstractmethod
    async def run(self) -> str:
        pass
    
    @abstractmethod
    def flow_content(self, visited: set) -> str:
        pass
    
    @abstractmethod
    def graph_struct(self, level: int) -> str:
        pass

    def add(self, action: 'BaseNode'):
        pass
        
class ActionNode(BaseNode):
    def __init__(self, next_text: str, node_name: str = ''):
        super().__init__()
        self.__llm = GPT()
        self.__prompt = None
        self._context = WorkflowContext()
        self._next_text = next_text
        self._node_name = node_name

    def __str__(self):
        if self._node_name:
            return self._node_name
        else:
            return super(ActionNode, ActionNode).__str__(self)
    
    @property
    def prompt(self):
        return self.__prompt

    @prompt.setter
    def prompt(self, value: str):
        if not isinstance(value, str):
            raise ValueError("prompt must be a string")
        self.__prompt = value

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    async def run(self) -> str:
        try:
            res = await self._run()
            res = self._process_response(res)
        except Exception as e:
            # self._context.log.format_message(format_log_message(str(e), "error"), "error")
            raise
        if self._next:
            return await self._next.run()
        else:
            return

    async def _run(self) -> str:
        # if not self.__prompt:
        #     raise ValueError("Prompt must not be empty")
        # res = await self._ask(self.__prompt)
        # return res
        self._logger.info(f"{str(self)}: running")


    async def _ask(self, prompt: str) -> str:
        result = await self.__llm.ask(prompt)
        # store PROMPT and RESULT in the log.md
        # make sure output them after __llm.ask(), for it's an asynchronize function
        # self._context.log.format_message(str(self),"action")
        # self._context.log.format_message(prompt,"prompt")
        # self._context.log.format_message(result,"response")
        return result
    
    def _process_response(self, response: str) -> str:
        return response
        
    def add(self, action: 'BaseNode'):
        self._logger.warn("ActionNode can't add other node")
    
    def flow_content(self, visited: set) -> str:
        if not self._next or self in visited: return ""
        visited.add(self)
        content = f"\t\t{str(self)} -->|{self._next_text}| {str(self._next)}\n"
        content += self._next.flow_content(visited)
        return content
    
    def graph_struct(self, level: int) -> str:
        return str(self)
        
class ActionLinkedList(BaseNode):
    def __init__(self, name: str, head: BaseNode):
        super().__init__()
        self.head = head
        self.__name = name

    def __str__(self):
        if self.__head:
            return str(self.__head)

    @property
    def head(self):
        return self.__head
    
    @head.setter
    def head(self, value):
        if isinstance(value, BaseNode):
            self.__head = value
            self.__tail = value
        else:
            raise TypeError("head must be a BaseNode")
        
    @property
    def _next(self):
        return self.__tail._next
    
    @_next.setter
    def _next(self, value):
        self.__tail._next = value

    def add(self, action: 'BaseNode'):
        if isinstance(action, BaseNode):
            self.__tail._next = action
            self.__tail = self.__tail._next
        else: 
            raise TypeError("element in ActionLinklist must be type of BaseNode")

    async def run(self, **kwargs):
        return await self.__head.run()
    
    def graph_struct(self, level: int) -> str:
        level += 1
        tables = "\t"
        content =  f"subgraph {self.__name}\n"
        node = self.__head
        while node and node != self.__tail:
            content += tables * (level)  + f"{node.graph_struct(level)}\n"
            node = node._next
        if node == self.__tail:
            content += tables * (level) + f"{node.graph_struct(level)}\n"

        content += tables * (level-1) + "end"
        return content
    
    def flow_content(self, visited: set) -> str:
        return self.__head.flow_content(visited)

if __name__ == "__main__":
    import asyncio
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
        pass
    
    def display_all(node: ActionNode):
        graph = node.graph_struct(level = 1)
        visited = set()
        res = node.flow_content(visited)
        text = f"""
```mermaid
    graph TD;
        Start((Start)) --> {str(node)}
{res}

    {graph}
```
        """
        return text

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
    # print(text)

    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(current_dir, 'flow.md')
    
    # 写入文本到文件
    with open(filepath, 'w') as file:
        file.write(text)

    # asyncio.run(main.run())
    # story = action.llm.ask("tell a story")
