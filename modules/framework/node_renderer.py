from abc import ABC, abstractmethod

class NodeRenderer(ABC):
    def set_node(self, node):
        self._node = node

    @abstractmethod
    def flow_content(self, visited: set) -> str:
        # Abstract method for generating flow content
        pass

    @abstractmethod
    def graph_struct(self, level: int) -> str:
        # Abstract method for generating graph structure
        pass

class ActionNodeRenderer(NodeRenderer):
    def flow_content(self, visited: set) -> str:
        # generating flow content in mermaid style
        if not self._node._next or self._node in visited:
            return ""
        visited.add(self._node)
        content = f"\t\t{str(self._node)} -->|{self._node._next_text}| {str(self._node._next)}\n"
        if self._node._error_handler:
            content += f"\t\t{str(self._node)} -->|failed| {str(self._node._error_handler)}\n"
            content += self._node._error_handler.display(visited)

        content += self._node._next._renderer.flow_content(visited)
        return content

    def graph_struct(self, level: int) -> str:
        # Method for generating graph structure
        return str(self._node)
    
class ActionLinkedListRenderer(NodeRenderer):
    def graph_struct(self, level: int) -> str:
        # Method for generating graph structure in mermaid style
        level += 1
        tables = "\t"
        content = f"subgraph {self._node._name}\n"
        node = self._node._head
        while node and node != self._node._tail:
            content += tables * (level) + f"{node._renderer.graph_struct(level)}\n"
            node = node._next
        if node == self._node._tail:
            content += tables * (level) + f"{node._renderer.graph_struct(level)}\n"

        content += tables * (level - 1) + "end"
        return content

    def flow_content(self, visited: set) -> str:
        return self._node._head._renderer.flow_content(visited)