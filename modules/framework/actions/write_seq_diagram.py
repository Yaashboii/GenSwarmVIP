from modules.framework.action import ActionNode
from modules.utils import parse_code
from modules.prompt.design_stage_prompt import WriteSeqDiagram_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES

class WriteSeqDiagram(ActionNode):
    def __init__(self, next_text: str , node_name: str = ''):
        super(WriteSeqDiagram, self).__init__(next_text, node_name)
        analysis = self._context.analysis.message
        function_content_list = [f['content'] for f in self._context.function_list]
        function_list_str = "\n".join(function_content_list)
        self.prompt = WriteSeqDiagram_PROMPT_TEMPLATE.format(
            analysis=analysis,
            robot_api=ROBOT_API,
            function_list=function_list_str,
            env_des=ENV_DES
        )

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response, lang='mermaid')
        code = f"```mermaid\n{code}\n```"
        self._context.sequence_diagram.message = code
        self._context.log.format_message(f"Write Sequence Diagram Success", 'success')
        return code