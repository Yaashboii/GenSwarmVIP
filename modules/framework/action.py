from abc import ABC, abstractmethod

from modules.framework.workflow_context import WorkflowContext
from modules.utils import setup_logger, LoggerLevel
from modules.llm.gpt import GPT
from tenacity import retry, stop_after_attempt, wait_random_exponential

class ActionNode(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)
        self._llm = GPT()
        self._next_actions = []
        self._prompt = None
        self._context = WorkflowContext()

    def __str__(self):
        return self.__class__.__name__
    
    @property
    def prompt(self):
        return self._prompt

    @prompt.setter
    def prompt(self, value: str):
        if not isinstance(value, str):
            raise TypeError("prompt must be a string")
        self._prompt = value

    @property
    def next_actions(self):
        return self._next_actions

    @next_actions.setter
    def next_actions(self, value):
        if not isinstance(value, list):
            raise TypeError("next_actions must be a list")
        elif len(value) == 0:
            raise ValueError("next_actions must not be empty")
        elif not all(isinstance(action, ActionNode) for action in value):
            raise TypeError("all elements in next_actions must be an instance of ActionNode")
        self._next_actions = value

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    async def run(self) -> str:
        try:
            res = await self._run()
            res = self._process_response(res)
        except Exception as e:
            # self._context.log.format_message(format_log_message(str(e), "error"), "error")
            raise
        return self._next_action()

    async def _run(self) -> str:
        if not self._prompt:
            raise ValueError("Prompt must not be empty")
        res = await self._ask(self._prompt)
        return res

    async def _ask(self, prompt: str) -> str:
        result = await self._llm.ask(prompt)
        # store PROMPT and RESULT in the log.md
        # make sure output them after _llm.ask(), for it's an asynchronize function
        # self._context.log.format_message(str(self),"action")
        # self._context.log.format_message(prompt,"prompt")
        # self._context.log.format_message(result,"response")
        return result
    
    def _process_response(self, response: str) -> str:
        return response

    def _next_action(self):
        if len(self._next_actions) == 1:
            return self._next_actions[0]
        else:
            return
        
    def add(self, action: 'ActionNode'):
        pass
    
    def flow_content(self, visited: set) -> str:
        if len(self._next_actions) == 0 or self in visited: return ""
        visited.add(self)
        content = "\n".join([f"{str(self)} --> {str(action)}\n" for action in self.next_actions])
        for child in self.next_actions:
            content += child.flow_content(visited)
        return content
    
    def graph_struct(self) -> str:
        return str(self)
        
class CompositeAction(ActionNode):
    def __init__(self, name: str):
        super().__init__()
        self._pipeline = []
        self._name = name

    def __str__(self):
        if len(self._pipeline) == 0: return
        return str(self._pipeline[0])

    def add(self, action: 'ActionNode'):
        if isinstance(action, ActionNode):
            self._pipeline.append(action)
        else: 
            raise TypeError("element in CompositeAction must be type of ActionNode")

    async def run(self, **kwargs):
        next_action = None
        for action in self._pipeline:
            next_action = action.run()
        return next_action
    
    def _next_action(self):
        if len(self._pipeline) == 0: return
        return self._pipeline[-1]._next_action()
    
    def graph_struct(self) -> str:
        content = ""
        if len(self._pipeline) == 0: return content
        content += f"subgraph {self._name} \n"
        content += "\n".join([action.graph_struct() for action in self._pipeline])
        content += "\nend\n"
        return content
    
    def flow_content(self, visited: set) -> str:
        if len(self._pipeline) == 0: return ''
        return self._pipeline[0].flow_content(visited)

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
    
    def display_all(node: ActionNode):
        graph = node.graph_struct()
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

    user_input = UserInput()
    write_analysis = WriteAnalyse()
    write_design = WriteDesign()
    write_code = WriteCode()
    write_test = WriteTest()

    user_input.next_actions = [write_analysis]
    write_analysis.next_actions = [write_design]
    write_design.next_actions = [write_code]
    a = CompositeAction("analysis and design")
    a.add(user_input)
    a.add(write_analysis)
    a.add(write_design)
    b = CompositeAction("code and test")
    b.add(write_code)
    b.add(write_test)
    write_code.next_actions = [write_test, write_design]
    c = CompositeAction("total")
    c.add(a)
    c.add(b)
    text = display_all(c)
    # print(text)

    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(current_dir, 'flow.md')

    print(b.next_actions)
    print(write_code.next_actions)

    # 写入文本到文件
    # with open(filepath, 'w') as file:
    #     file.write(text)
    # asyncio.run(action.run())
    # story = action.llm.ask("tell a story")
