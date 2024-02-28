from modules.actions.action import Action
from modules.utils import parse_code


class WriteSeqDiagram(Action):
    name: str = "WriteSeqDiagram"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response, lang='mermaid')
        self._context.sequence_diagram.message = code
        self._logger.info(f"Write Sequence Diagram Success")
        code = f"```mermaid\n{code}\n```"
        return code
