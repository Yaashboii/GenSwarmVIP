from modules.actions.action import Action
from modules.utils import parse_code


class WriteSeqDiagram(Action):
    name: str = "WriteSeqDiagram"

    def process_response(self, response: str, **kwargs) -> str:
        code = parse_code(text=response, lang='mermaid')
        code = f"```mermaid\n{code}\n```"
        self._context.sequence_diagram.message = code
        self._context.log.format_message(f"Write Sequence Diagram Success", 'success')
        return code
