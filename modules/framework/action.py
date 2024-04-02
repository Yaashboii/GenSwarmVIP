from abc import ABC, abstractmethod

from tenacity import retry, stop_after_attempt, wait_random_exponential

from modules.framework.workflow_context import WorkflowContext
from modules.utils import setup_logger, LoggerLevel
from modules.llm.gpt import GPT
from modules.framework.code_error import CodeError

class BaseNode(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)
        self.__next = None # next node

    def __str__(self):
        return self.__class__.__name__

    @property
    def _next(self):
        # _ means this is protected property
        return self.__next
    
    @_next.setter
    def _next(self, value):
        if not isinstance(value, BaseNode):
            raise ValueError("Value must be a BaseNode")
        self.__next = value

    @abstractmethod
    async def run(self) -> str:
        # Abstract method for executing node logic
        pass
    
    @abstractmethod
    def flow_content(self, visited: set) -> str:
        # Abstract method for generating flow content
        pass
    
    @abstractmethod
    def graph_struct(self, level: int) -> str:
        # Abstract method for generating graph structure
        pass

    def add(self, action: 'BaseNode'):
        # Method for adding action to node
        # It also provides same interface for both ActinoNode and LinkedListNode
        pass
        
class ActionNode(BaseNode):
    def __init__(self, next_text: str, node_name: str = ''):
        super().__init__()
        self.__llm = GPT()
        self.__prompt = None
        self._context = WorkflowContext()
        self._next_text = next_text # label text rendered in mermaid graph
        self._node_name = node_name # to distinguish objects of same class type
        self._error_handler = None # this is a chain of handlers, see handler.py

    def __str__(self):
        if self._node_name:
            return self._node_name
        else:
            # return class name when node_name is not defined
            return super(ActionNode, ActionNode).__str__(self)
    
    @property
    def prompt(self):
        return self.__prompt

    @prompt.setter
    def prompt(self, value: str):
        if not isinstance(value, str):
            raise ValueError("prompt must be a string")
        self.__prompt = value

    @property
    def error_handler(self):
        raise AttributeError("This property is write-only")

    @error_handler.setter
    def error_handler(self, value: 'Handler'):
        self._error_handler = value

    def add(self, action: 'BaseNode'):
        self._logger.warn("ActionNode can't add other node")
    
    def flow_content(self, visited: set) -> str:
        # generating flow content in mermaid style
        if not self._next or self in visited:
            return ""
        visited.add(self)
        content = f"\t\t{str(self)} -->|{self._next_text}| {str(self._next)}\n"
        if self._error_handler:
            content += f"\t\t{str(self)} -->|failed| {str(self._error_handler)}\n"
            content += self._error_handler.display(visited)

        content += self._next.flow_content(visited)
        return content
    
    def graph_struct(self, level: int) -> str:
        # Method for generating graph structure
        return str(self)
    
    def _can_skip(self) -> bool:
        return False
    
    def _build_prompt(self):
        pass

    @retry(wait=wait_random_exponential(min=1, max=60), stop=stop_after_attempt(6))
    async def run(self) -> str:
        try:
            # First create a prompt, then utilize it to query the language model.
            self._build_prompt()
            if not self._can_skip():
                res = await self._run()
                res = self._process_response(res)
                if isinstance(res, CodeError):
                    # If response is CodeError, handle it and move to next action
                    if self._error_handler:
                        next_action = self._error_handler.handle(res)
                        return await next_action.run()
                    else:
                        raise ValueError("No error handler available to handle request")
            if self._next is not None:
                return await self._next.run()
        except Exception as e:
            raise

    async def _run(self) -> str:
        if self.__prompt is None:
            raise SystemExit("Prompt is required")
        code = await self._ask(self.__prompt)
        return code

    async def _ask(self, prompt: str) -> str:
        result = await self.__llm.ask(prompt)
        return result
    
    def _process_response(self, response: str) -> str:
        return response
    
class ActionLinkedList(BaseNode):
    def __init__(self, name: str, head: BaseNode):
        super().__init__()
        self.head = head # property is used
        self.__name = name # name of the structure

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
            # add new node\linked list to this linked list
            self.__tail._next = action
            self.__tail = self.__tail._next
        else: 
            raise TypeError("element in ActionLinklist must be type of BaseNode")

    async def run(self, **kwargs):
        return await self.__head.run()
    
    def graph_struct(self, level: int) -> str:
        # Method for generating graph structure in mermaid style
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


def display_all(node: ActionNode, error_handler):
    graph = node.graph_struct(level = 1)
    visited = set()
    res = node.flow_content(visited)
    text = f"""
```mermaid
graph TD;
    Start((Start)) --> {str(node)}
{res}

{graph}

subgraph chain of handlers
{error_handler.struct()}
end
```
    """
    return _clean_graph(text)

def _clean_graph(graph: str):
    lines = set()
    unique_lines = []
    for line in graph.split('\n'):
        content = line.strip()
        if content not in lines or content == "end":
            unique_lines.append(line)
            lines.add(line.strip())

    return '\n'.join(unique_lines)


if __name__ == "__main__":
    pass