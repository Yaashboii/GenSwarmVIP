import traceback
from abc import ABC, abstractmethod

from tenacity import retry, stop_after_attempt, wait_random_exponential

from modules.framework.context import WorkflowContext
from modules.utils import setup_logger, LoggerLevel, root_manager
from modules.llm.gpt import GPT
from modules.framework.code_error import CodeError
from modules.framework.node_renderer import *
from modules.framework.context import logger

class BaseNode(ABC):
    def __init__(self):
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)
        self.__next = None  # next node
        self._renderer = None

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
    
    def _set_renderer(self, renderer):
        self._renderer = renderer
        renderer.set_node(self)

    def flow_content(self, visited):
        return self._renderer.flow_content(visited)
    
    def graph_struct(self, level):
        return self._renderer.graph_struct(level)

class ActionNode(BaseNode):
    def __init__(self, next_text: str, node_name: str = ''):
        super().__init__()
        self.__llm = GPT()
        self.__prompt = None
        self._context = WorkflowContext()
        self._next_text = next_text  # label text rendered in mermaid graph
        self._node_name = node_name  # to distinguish objects of same class type
        self._error_handler = None  # this is a chain of handlers, see handler.py
        self._set_renderer(ActionNodeRenderer())

    def __str__(self):
        if self._node_name:
            return self._node_name
        else:
            # return class name when node_name is not defined
            return super(ActionNode, ActionNode).__str__(self)

    @property
    def context(self):
        return self._context

    @context.setter
    def context(self, value: WorkflowContext):
        if not isinstance(value, WorkflowContext):
            raise ValueError("context must be a WorkflowContext")
        self._context = value

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

    def _can_skip(self) -> bool:
        return False

    def _build_prompt(self):
        pass

    async def run(self) -> str:
        # First create a prompt, then utilize it to query the language model.
        self._build_prompt()
        logger.log(f"Action: {str(self)}", "info")
        if not self._can_skip():
            res = await self._run()
            self.context.save_to_file(file_path=root_manager.workspace_root / f"{self}.pkl")
            if isinstance(res, CodeError):
                # If response is CodeError, handle it and move to next action
                if self._error_handler:
                    next_action = self._error_handler.handle(res)
                    return await next_action.run()
                else:
                    raise ValueError("No error handler available to handle request")
        if self._next is not None:
            return await self._next.run()

    @retry(stop=stop_after_attempt(5), wait=wait_random_exponential(multiplier=1, max=10))
    async def _run(self) -> str:
        try:
            if self.__prompt is None:
                raise SystemExit("Prompt is required")
            code = await self._ask(self.__prompt)
            logger.log(f"Action: {str(self)}", "info")
            logger.log(f"Prompt:\n {self.__prompt}", "debug")
            logger.log(f"Response:\n {code}", "info")
            code = self._process_response(code)
            return code
        except Exception as e:
            tb = traceback.format_exc()
            logger.log(f"Error in {str(self)}: {e},\n {tb}", "error")
            raise Exception

    async def _ask(self, prompt: str) -> str:
        result = await self.__llm.ask(prompt)
        return result

    def _process_response(self, response: str) -> str:
        return response


class ActionLinkedList(BaseNode):
    def __init__(self, name: str, head: BaseNode):
        super().__init__()
        self.head = head  # property is used
        self._name = name  # name of the structure
        self._set_renderer(ActionLinkedListRenderer())

    def __str__(self):
        if self._head:
            return str(self._head)

    @property
    def head(self):
        return self._head

    @head.setter
    def head(self, value):
        if isinstance(value, BaseNode):
            self._head = value
            self._tail = value
        else:
            raise TypeError("head must be a BaseNode")

    @property
    def _next(self):
        return self._tail._next

    @_next.setter
    def _next(self, value):
        self._tail._next = value

    def add(self, action: 'BaseNode'):
        if isinstance(action, BaseNode):
            self._tail._next = action
            self._tail = action
        else:
            raise ValueError("Value must be a BaseNode")

    async def run(self, **kwargs):
        return await self._head.run()


def display_all(node: ActionNode, error_handler):
    graph = node.graph_struct(level=1)
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
